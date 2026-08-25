use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::OnceLock;

use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::hal::gpio::{InputPin, OutputPin};
use esp_idf_svc::hal::sd::mmc::{SdMmc, SdMmcHostDriver};
use esp_idf_svc::hal::sd::{SdCardConfiguration, SdCardDriver};
use esp_idf_svc::io::vfs::MountedFatfs;
use esp_idf_svc::sys::EspError;
use log::{error, info};
use parking_lot::Mutex;

const MOUNT_POINT: &str = "/sdcard";
const LOG_EXT: &str = ".bin";

static SD: OnceLock<Mutex<SdCard>> = OnceLock::new();

type Vfs = MountedFatfs<Fatfs<SdCardDriver<SdMmcHostDriver<'static>>>>;

pub struct SdCard {
    current_file: Option<File>,
    write_buf: [u8; 512],
    write_buf_len: usize,
    log_name: String,
    _vfs: Vfs,
}

impl SdCard {
    pub fn init(
        slot: impl SdMmc + 'static,
        cmd: impl OutputPin + 'static,
        clk: impl OutputPin + 'static,
        d0: impl InputPin + OutputPin + 'static,
        d1: impl InputPin + OutputPin + 'static,
        d2: impl InputPin + OutputPin + 'static,
        d3: impl InputPin + OutputPin + 'static,
        cd: Option<impl InputPin + 'static>,
        wp: Option<impl InputPin + 'static>,
        config: &SdCardConfiguration,
    ) -> Result<(), EspError> {
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
        let card = Self::build(host, config)?;

        if SD.set(Mutex::new(card)).is_err() {
            error!("SD card already initialized");
            return Err(EspError::from_infallible::<-1>());
        }

        Ok(())
    }

    fn build(
        host: SdMmcHostDriver<'static>,
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

    fn instance() -> Result<&'static Mutex<SdCard>, EspError> {
        SD.get().ok_or_else(|| {
            error!("SD card not initialized");
            EspError::from_infallible::<-1>()
        })
    }

    /// Append bytes to the current log. Synced to storage on every call.
    pub fn write(data: &[u8]) -> Result<(), EspError> {
        Self::instance()?.lock().write_buffered(data)
    }

    /// Flush any buffered data without closing the file.
    pub fn sync() -> Result<(), EspError> {
        Self::instance()?.lock().sync_buffer()
    }

    /// Flush, close the current log and open the next numbered one.
    pub fn next_log() -> Result<(), EspError> {
        Self::instance()?.lock().roll()
    }

    /// List all `.bin` files on the card.
    pub fn list_logs() -> Vec<String> {
        Self::read_dir_logs()
    }

    /// Size in bytes of a named file. Returns 0 if not found.
    pub fn file_size(name: &str) -> u64 {
        fs::metadata(PathBuf::from(MOUNT_POINT).join(name))
            .map(|m| m.len())
            .unwrap_or(0)
    }

    /// File name of the current log, or `None` if the card isn't initialized.
    pub fn current_name() -> Option<String> {
        SD.get().map(|sd| sd.lock().name())
    }

    /// Stream the current log in `chunk_size`-byte chunks via `cb`.
    pub fn stream_current(chunk_size: usize, cb: impl FnMut(&[u8])) {
        match SD.get() {
            Some(sd) => sd.lock().stream_current_impl(chunk_size, cb),
            None => error!("SD card not initialized"),
        }
    }

    /// Stream any named file in `chunk_size`-byte chunks via `cb`.
    pub fn stream_file(name: &str, chunk_size: usize, mut cb: impl FnMut(&[u8])) {
        Self::stream_file_impl(name, chunk_size, &mut cb);
    }

    fn write_buffered(&mut self, data: &[u8]) -> Result<(), EspError> {
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

    fn sync_buffer(&mut self) -> Result<(), EspError> {
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

    fn roll(&mut self) -> Result<(), EspError> {
        self.sync_buffer()?;
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

    fn name(&self) -> String {
        format!("{}{LOG_EXT}", self.log_name)
    }

    fn stream_current_impl(&mut self, chunk_size: usize, mut cb: impl FnMut(&[u8])) {
        self.sync_buffer().ok();
        let name = self.name();
        Self::stream_file_impl(&name, chunk_size, &mut cb);
    }

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
}

impl Drop for SdCard {
    fn drop(&mut self) {
        self.sync_buffer().ok();
    }
}

// SAFETY: all access goes through `SD`'s `Mutex`, so the driver's internal
// pointers are never touched by more than one thread at a time.
unsafe impl Send for SdCard {}
