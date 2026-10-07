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

    #[error("{0}")]
    Meshes(String),

    #[error("replicar simulates soccar; this replay is {0}")]
    UnsupportedMode(String),

    #[error("the file could not be read: {0}")]
    Read(#[from] replicar_format::ReadError),

    #[error("{0}")]
    Io(String),

    #[error("cannot resimulate: {0}")]
    Resimulation(String),

    #[error("the file could not be written: {0}")]
    Write(#[from] replicar_format::WriteError),

    #[error(
        "the conversion has {built:?} rows, not the {asked:?} the file asks for (set the converter's rows)"
    )]
    RowsNotBuilt {
        built: replicar_format::RowRate,
        asked: replicar_format::RowRate,
    },

    #[error("{frame} has an invalid time {time}")]
    InvalidTime {
        frame: replicar_format::FrameIndex,
        time: f32,
    },
}
