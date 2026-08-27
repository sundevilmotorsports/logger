use std::time::Duration;

const RETRY_INTERVAL: Duration = Duration::from_secs(3);

/// Calls `attempt` forever. Each call should initialize the sensor and then run
/// until something goes wrong
pub fn run<F, E>(mut attempt: F) -> !
where
    F: FnMut() -> Result<(), E>,
    E: std::fmt::Debug,
{
    let name = std::any::type_name::<F>();
    loop {
        if let Err(e) = attempt() {
            log::error!("{name} failed, restarting: {e:?}");
            std::thread::sleep(RETRY_INTERVAL);
        }
    }
}
