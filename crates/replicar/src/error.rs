//! The one error type of the library.

/// Why a replay could not be converted.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("the replay could not be parsed: {0}")]
    Parse(#[from] boxcars::ParseError),

    #[error("the replay has no network frames, so there is nothing to reconstruct")]
    NoNetworkFrames,

    #[error("the replay has {0} frames, more than a frame index can count")]
    TooManyFrames(usize),
}
