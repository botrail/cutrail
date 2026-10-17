//! Error types.

/// Errors reported by cutrail.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// Invalid input (tool, path or stock).
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// Not supported in this version.
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// A kernel operation failed. The state has not been modified.
    #[error("kernel failure: {0}")]
    Kernel(String),
    /// The result failed validation. The state has not been modified.
    #[error("validation failed: {0}")]
    Validation(String),
    /// An I/O operation failed.
    #[error("io error: {0}")]
    Io(String),
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e.to_string())
    }
}

/// `Result` with cutrail's error type.
pub type Result<T> = std::result::Result<T, Error>;
