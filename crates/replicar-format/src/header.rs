//! The file's header: JSON in the Parquet key-value metadata under `replicar` (docs/glossary.md, "replicar
//! file"). It says what the file is and what it holds, and has what is per replay rather than per frame.

use serde::{Deserialize, Serialize};

/// The file format's version: a reader refuses a file of a later version.
pub const FORMAT_VERSION: u32 = 1;

/// The key of the header in the Parquet key-value metadata.
pub const HEADER_KEY: &str = "replicar";

/// A player of the replay, at its player index.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlayerInfo {
    pub index: u8,
    /// The player's identity across the replay (platform id, else player id, else actor).
    pub key: String,
    pub name: Option<String>,
    /// 0 blue, 1 orange.
    pub team: u8,
    pub body_product_id: Option<u32>,
    /// RocketSim's car body configuration for the player.
    pub hitbox: String,
}

/// A boost pad of the arena, at its pad index.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PadInfo {
    pub position: [f32; 3],
    pub is_big: bool,
}

/// A play segment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SegmentInfo {
    pub first_frame: u32,
    pub last_frame: u32,
    /// Future-derived: how it ends (`SegmentEnd::name`).
    pub end: String,
}

/// The header.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub format_version: u32,
    /// The SHA-256 of the replay file, hex.
    pub replay_sha256: String,
    pub replicar_version: String,
    pub rocketsim_version: String,
    /// The column groups written, the state's precision, and whether frames outside play are included.
    pub groups: Vec<String>,
    pub precision: String,
    pub all_frames: bool,
    pub players: Vec<PlayerInfo>,
    pub pads: Vec<PadInfo>,
    pub segments: Vec<SegmentInfo>,
    /// The final scores as the replay's last frame shows them (blue, orange): the match result, which never
    /// reaches a frame.
    pub final_scores: [Option<i32>; 2],
    /// The conversion's configuration and what it counted, as the converter reports them.
    pub configuration: serde_json::Value,
    pub diagnostics: serde_json::Value,
}
