# Car-body hitbox catalog

`body_products.csv` is the 238 `Body` rows extracted from the user's headerless
`D:/rokutleg/replays/items.csv` (provided as the August 2026 item catalog). The
source file's SHA-256 was
`d67f060b1964171a81875fc6ce77b9f141909bc989f5883ffec5aacf968a6713`.
The source columns were product ID, category, game asset path, and display name.
Only the Body rows are committed, so later agents do not need access to the
external drive to reproduce this mapping.

`official_hitboxes.tsv` is a name-and-family snapshot of [Epic/Psyonix's car
hitbox roster](https://www.epicgames.com/help/c-37599050/c-32343914/a20257614?lang=en-US),
read on 2026-09-27. A row with slashes lists several named variants in one
family. `body_hitbox_aliases.tsv` records reviewed name differences between that
roster and the product catalog. The generator accepts only exact matches after
Unicode and punctuation normalization or these explicit aliases. It does not
guess from model shape, asset slug, or a similar product name.

`body_hitbox_additions.tsv` records bodies documented outside the English
roster, with a source URL per assignment. The Psyclops row uses the [pinned
RocketSim body preset](https://github.com/ZealanL/RocketSim/blob/79f4d22fc533614d540b88457a96352c17da6b73/rocketsim/src/sim/car/car_body_config.rs)
because that special body is absent from the six-family support roster.

Run `python scripts/build_body_hitboxes.py` to regenerate
`body_hitboxes.tsv`, or `python scripts/build_body_hitboxes.py --check` to
verify it. The generated table contains every Body product ID. `unmapped`
in the `hitbox` column marks an unresolved row; conversion preserves its raw
product ID and uses the Octane fallback if it appears on a playing car. As of this
snapshot, 213 of 238 Body rows are mapped. Eleven of the 25 unresolved rows
are generic drops or mystery items labeled `Body` in the source CSV; the
other fourteen are named vehicles absent from the checked authoritative
hitbox sources. The English roster appears to lag some 2026 releases, so
extend the catalog when a direct source or in-game inspection is available.
