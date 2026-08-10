use log::info;
use parking_lot::Mutex;
use std::collections::HashMap;
use std::convert::Infallible;
use std::sync::LazyLock;

const LOG_EXT: &str = ".bin";

pub static SD: LazyLock<Mutex<FakeSdCard>> = LazyLock::new(|| Mutex::new(FakeSdCard::new()));

pub struct FakeSdCard {
    files: HashMap<String, Vec<u8>>,
    log_name: String,
}

impl FakeSdCard {
    pub fn new() -> Self {
        let log_name = "0001".to_string();
        let mut files = HashMap::new();
        files.insert(format!("{log_name}{LOG_EXT}"), Vec::new());
        info!("fake SD: logging to {log_name}{LOG_EXT}");
        Self { files, log_name }
    }

    pub fn next_log(&mut self) -> Result<(), Infallible> {
        let next: u32 = self.log_name.parse().unwrap_or(0) + 1;
        self.log_name = format!("{next:04}");
        self.files.entry(self.current_name()).or_default();
        info!("fake SD: logging to {}", self.current_name());
        Ok(())
    }

    pub fn list_logs(&self) -> Vec<String> {
        self.files.keys().cloned().collect()
    }

    pub fn file_size(&self, name: &str) -> u64 {
        self.files.get(name).map_or(0, |b| b.len() as u64)
    }

    pub fn current_name(&self) -> String {
        format!("{}{LOG_EXT}", self.log_name)
    }

    pub fn stream_current(&mut self, chunk_size: usize, cb: impl FnMut(&[u8])) {
        let name = self.current_name();
        self.stream_file(&name, chunk_size, cb);
    }

    pub fn stream_file(&self, name: &str, chunk_size: usize, mut cb: impl FnMut(&[u8])) {
        let Some(data) = self.files.get(name) else {
            return;
        };
        for chunk in data.chunks(chunk_size.max(1)) {
            cb(chunk);
        }
    }

    /// Bytes of `name` from `offset`; `None` if missing, `Some(&[])` at EOF.
    pub fn read_chunk(&self, name: &str, offset: u64, len: usize) -> Option<&[u8]> {
        let data = self.files.get(name)?;
        let start = (offset as usize).min(data.len());
        Some(&data[start..(start + len).min(data.len())])
    }
}

impl std::io::Write for FakeSdCard {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let name = self.current_name();
        self.files.entry(name).or_default().extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
