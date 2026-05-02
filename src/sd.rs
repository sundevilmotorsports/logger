use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::PathBuf;

use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use esp_idf_svc::hal::sd::mmc::{SdMmc, SdMmcHostDriver};
use esp_idf_svc::hal::sd::{SdCardConfiguration, SdCardDriver};
use esp_idf_svc::io::vfs::MountedFatfs;
use esp_idf_svc::sys::EspError;
use log::{error, info};

const MOUNT_POINT: &str = "/sdcard";
const LOG_EXT: &str = ".bin";

type Vfs<'d> = MountedFatfs<Fatfs<SdCardDriver<SdMmcHostDriver<'d>>>>;

pub struct SdCard<'d> {
    current_file: Option<File>,
    write_buf: [u8; 512],
    write_buf_len: usize,
    log_name: String,
    _vfs: Vfs<'d>,
}

impl<'d> SdCard<'d> {
    pub fn new(
        slot: impl SdMmc + 'd,
        cmd: impl OutputPin + 'd,
        clk: impl OutputPin + 'd,
        d0: impl InputPin + OutputPin + 'd,
        d1: impl InputPin + OutputPin + 'd,
        d2: impl InputPin + OutputPin + 'd,
        d3: impl InputPin + OutputPin + 'd,
        cd: Option<impl InputPin + 'd>,
        wp: Option<impl InputPin + 'd>,
        config: &SdCardConfiguration,
    ) -> Result<Self, EspError> {
        let host = SdMmcHostDriver::new_4bits(
            slot,
            cmd,
            clk,
            d0,
            d1,
            d2,
            d3,
            cd,
            wp,
            &Default::default(),
        )?;
        Self::from_host(host, config)
    }

    fn from_host(
        host: SdMmcHostDriver<'d>,
        config: &SdCardConfiguration,
    ) -> Result<Self, EspError> {
        let card = SdCardDriver::new_mmc(host, config)?;
        let fatfs = Fatfs::new_sdcard(0, card)?;
        let vfs = MountedFatfs::mount(fatfs, MOUNT_POINT, 4)?;

        info!("SD card mounted at {MOUNT_POINT}");

        let log_name = format!("{:04}", Self::next_index());
        let path = Self::log_path(&log_name);
        info!("Logging to {path:?}");
        let file = File::create(&path).map_err(|e| {
            error!("Failed to create {path:?}: {e}");
            EspError::from_infallible::<-1>()
        })?;

        Ok(Self {
            current_file: Some(file),
            write_buf: [0u8; 512],
            write_buf_len: 0,
            log_name,
            _vfs: vfs,
        })
    }

    // region writes

    /// Append bytes to the current log. Synced to storage on every call.
    pub fn write(&mut self, data: &[u8]) -> Result<(), EspError> {
        if self.current_file.is_none() {
            return Err(EspError::from_infallible::<-1>());
        }

        let mut pos = 0;
        while pos < data.len() {
            let space = self.write_buf.len() - self.write_buf_len;
            let chunk = (data.len() - pos).min(space);
            self.write_buf[self.write_buf_len..self.write_buf_len + chunk]
                .copy_from_slice(&data[pos..pos + chunk]);
            self.write_buf_len += chunk;
            pos += chunk;

            if self.write_buf_len == self.write_buf.len() {
                {
                    let f = self.current_file.as_mut().unwrap();
                    Self::write_and_sync(f, &self.write_buf)?;
                }
                self.write_buf_len = 0;
            }
        }

        if self.write_buf_len > 0 {
            let len = self.write_buf_len;
            {
                let f = self.current_file.as_mut().unwrap();
                Self::write_and_sync(f, &self.write_buf[..len])?;
            }
            self.write_buf_len = 0;
        }

        Ok(())
    }

    /// Flush any buffered data without closing the file.
    pub fn sync(&mut self) -> Result<(), EspError> {
        if self.write_buf_len > 0 {
            let len = self.write_buf_len;
            let f = self
                .current_file
                .as_mut()
                .ok_or(EspError::from_infallible::<-1>())?;
            Self::write_and_sync(f, &self.write_buf[..len])?;
            self.write_buf_len = 0;
        }
        Ok(())
    }

    // endregion
    // region log management

    /// Flush, close the current log and open the next numbered one.
    pub fn next_log(&mut self) -> Result<(), EspError> {
        self.sync()?;
        self.current_file = None;

        self.log_name = format!("{:04}", Self::next_index());
        let path = Self::log_path(&self.log_name);
        info!("Logging to {path:?}");
        self.current_file = Some(File::create(&path).map_err(|e| {
            error!("Failed to create {path:?}: {e}");
            EspError::from_infallible::<-1>()
        })?);

        Ok(())
    }

    /// List all `.bin` files on the card.
    pub fn list_logs(&self) -> Vec<String> {
        Self::read_dir_logs()
    }

    /// Size in bytes of a named file. Returns 0 if not found.
    pub fn file_size(&self, name: &str) -> u64 {
        fs::metadata(PathBuf::from(MOUNT_POINT).join(name))
            .map(|m| m.len())
            .unwrap_or(0)
    }

    /// File name of the current log
    pub fn current_name(&self) -> String {
        format!("{}{LOG_EXT}", self.log_name)
    }
    // endregion
    // region streaming reads

    /// Stream the current log in `chunk_size`-byte chunks via `cb`.
    pub fn stream_current(&mut self, chunk_size: usize, mut cb: impl FnMut(&[u8])) {
        self.sync().ok();
        let name = self.current_name();
        Self::stream_file_impl(&name, chunk_size, &mut cb);
    }

    /// Stream any named file in `chunk_size`-byte chunks via `cb`.
    pub fn stream_file(&self, name: &str, chunk_size: usize, mut cb: impl FnMut(&[u8])) {
        Self::stream_file_impl(name, chunk_size, &mut cb);
    }

    // endregion
    // region private

    fn write_and_sync(file: &mut File, data: &[u8]) -> Result<(), EspError> {
        file.write_all(data).map_err(|e| {
            error!("Write failed: {e}");
            EspError::from_infallible::<-1>()
        })?;
        file.sync_all().map_err(|e| {
            error!("Sync failed: {e}");
            EspError::from_infallible::<-1>()
        })
    }

    fn stream_file_impl(name: &str, chunk_size: usize, cb: &mut impl FnMut(&[u8])) {
        let path = PathBuf::from(MOUNT_POINT).join(name);
        let Ok(mut file) = File::open(&path) else {
            return;
        };
        let mut buf = vec![0u8; chunk_size];
        loop {
            match file.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => cb(&buf[..n]),
            }
        }
    }

    fn read_dir_logs() -> Vec<String> {
        let Ok(dir) = fs::read_dir(MOUNT_POINT) else {
            return Vec::new();
        };
        dir.filter_map(|e| {
            let e = e.ok()?;
            if !e.file_type().ok()?.is_file() {
                return None;
            }
            e.file_name().into_string().ok()
        })
        .collect()
    }

    // "XXXX.bin" -> XXXX as u32; rejects any other format.
    fn parse_index(name: &str) -> Option<u32> {
        let u = name.to_uppercase();
        (u.len() == 8 && u.ends_with(".BIN")).then(|| u[..4].parse().ok())?
    }

    fn next_index() -> u32 {
        Self::read_dir_logs()
            .iter()
            .filter_map(|n| Self::parse_index(n))
            .max()
            .map_or(1, |m| m + 1)
    }

    fn log_path(base: &str) -> PathBuf {
        PathBuf::from(MOUNT_POINT).join(format!("{base}{LOG_EXT}"))
    }

    // endregion
}

impl Drop for SdCard<'_> {
    fn drop(&mut self) {
        self.sync().ok();
    }
}
