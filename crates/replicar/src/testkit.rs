//! Synthetic replays for unit tests: a decoded network feed built directly, without a replay file.

use replicar_format::FrameIndex;

use crate::decode::{
    DecodeDiagnostics, GameState, NetworkBody, NetworkFrame, NetworkReplay, NetworkValue,
    ReplayHeader, ValueSource,
};

/// A value the replay sent at `frame`.
pub(crate) fn sent<T>(value: T, frame: u32) -> NetworkValue<T> {
    NetworkValue {
        value,
        frame: FrameIndex(frame),
        source: ValueSource::Replay,
    }
}

/// An empty frame in play at `time` seconds.
pub(crate) fn frame(index: u32, time: f32, delta: f32) -> NetworkFrame {
    NetworkFrame {
        index: FrameIndex(index),
        time,
        delta,
        ball: None,
        cars: Vec::new(),
        players: Vec::new(),
        team_scores: [None, None],
        seconds_remaining: None,
        overtime: None,
        game_state: Some(sent(GameState::Active, index)),
        events: Vec::new(),
        pad_records: Vec::new(),
    }
}

/// A body updated at `frame` with a position and a linear velocity.
pub(crate) fn moving_body(frame: u32, position: [f32; 3], velocity: [f32; 3]) -> NetworkBody {
    NetworkBody {
        position: Some(sent(position, frame)),
        linear_velocity: Some(sent(velocity, frame)),
        ..NetworkBody::default()
    }
}

/// A soccar replay of the given frames.
pub(crate) fn replay(frames: Vec<NetworkFrame>) -> NetworkReplay {
    NetworkReplay {
        header: ReplayHeader {
            game_type: "TAGame.Replay_Soccar_TA".to_owned(),
            levels: Vec::new(),
            final_scores: [None, None],
            counted_stats: Vec::new(),
        },
        frames,
        diagnostics: DecodeDiagnostics::default(),
    }
}
