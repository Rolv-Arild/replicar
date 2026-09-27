"""Build the checked-in body-ID map from the supplied product and official roster snapshots.

Run from any directory with ``python scripts/build_body_hitboxes.py``. Use ``--check``
to verify that the committed map is current. Matching is exact after harmless
Unicode/punctuation normalization; unmatched names require reviewed aliases.
"""

from __future__ import annotations

import argparse
import csv
import io
import re
import unicodedata
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRODUCTS = ROOT / "data/body_products.csv"
OFFICIAL = ROOT / "data/official_hitboxes.tsv"
ALIASES = ROOT / "data/body_hitbox_aliases.tsv"
ADDITIONS = ROOT / "data/body_hitbox_additions.tsv"
OUTPUT = ROOT / "data/body_hitboxes.tsv"


def normalized(name: str) -> str:
    name = unicodedata.normalize("NFKD", name.casefold())
    name = name.replace("&", "and")
    return "".join(character for character in name if character.isalnum())


def read_rows(path: Path, delimiter: str) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file, delimiter=delimiter))


def official_names() -> dict[str, tuple[str, str]]:
    names: dict[str, tuple[str, str]] = {}
    for row in read_rows(OFFICIAL, "\t"):
        family = row["hitbox"]
        if family not in {"breakout", "dominus", "hybrid", "merc", "octane", "plank"}:
            raise ValueError(f"unsupported official family: {family}")
        for name in row["name"].split("/"):
            name = name.strip()
            # Platform qualifiers on the official page are not part of body names.
            name = re.sub(r"\s*\((?:Xbox|PlayStation|Nintendo) Exclusive\)$", "", name)
            key = normalized(name)
            previous = names.get(key)
            if previous is not None and previous[1] != family:
                raise ValueError(f"conflicting official hitboxes for {name}: {previous[1]}, {family}")
            names[key] = (name, family)
    return names


def build() -> tuple[str, list[str]]:
    official = official_names()
    aliases: dict[int, str] = {}
    for row in read_rows(ALIASES, "\t"):
        product_id = int(row["id"])
        if product_id in aliases:
            raise ValueError(f"duplicate alias ID: {product_id}")
        aliases[product_id] = row["official_name"]
        if normalized(row["official_name"]) not in official:
            raise ValueError(f"alias {product_id} has no official roster match: {row['official_name']}")

    additions: dict[int, tuple[str, str, str]] = {}
    for row in read_rows(ADDITIONS, "\t"):
        product_id = int(row["id"])
        if product_id in additions or product_id in aliases:
            raise ValueError(f"duplicate or conflicting documented ID: {product_id}")
        if row["hitbox"] not in {"breakout", "dominus", "hybrid", "merc", "octane", "plank", "psyclops"}:
            raise ValueError(f"unsupported documented hitbox: {row['hitbox']}")
        if not row["source_url"].startswith("https://"):
            raise ValueError(f"missing authoritative URL for {product_id}")
        additions[product_id] = (row["name"], row["hitbox"], row["source_url"])

    product_rows = read_rows(PRODUCTS, ",")
    ids: set[int] = set()
    output = io.StringIO(newline="")
    writer = csv.writer(output, delimiter="\t", lineterminator="\n")
    writer.writerow(["id", "name", "hitbox", "official_name"])
    unresolved = []
    for row in sorted(product_rows, key=lambda row: int(row["id"])):
        product_id = int(row["id"])
        if product_id in ids:
            raise ValueError(f"duplicate body ID: {product_id}")
        ids.add(product_id)
        addition = additions.get(product_id)
        if addition is not None and normalized(addition[0]) != normalized(row["name"]):
            raise ValueError(f"documented body name differs for {product_id}: {addition[0]}")
        lookup_name = aliases.get(product_id, row["name"])
        matched = official.get(normalized(lookup_name))
        if addition is not None:
            if matched is not None and matched[1] != addition[1]:
                raise ValueError(f"conflicting official hitbox for {product_id}")
            matched = (row["name"], addition[1])
        if matched is None:
            unresolved.append(f"{product_id}\t{row['name']}")
            writer.writerow([product_id, row["name"], "unmapped", "-"])
        else:
            writer.writerow([product_id, row["name"], matched[1], matched[0]])
    extra_aliases = set(aliases) - ids
    if extra_aliases:
        raise ValueError(f"aliases refer to absent body IDs: {sorted(extra_aliases)}")
    extra_additions = set(additions) - ids
    if extra_additions:
        raise ValueError(f"documented hitboxes refer to absent body IDs: {sorted(extra_additions)}")
    return output.getvalue(), unresolved


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if the committed map is stale")
    args = parser.parse_args()
    content, unresolved = build()
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text(encoding="utf-8") != content:
            raise SystemExit(f"stale map: regenerate {OUTPUT}")
    else:
        OUTPUT.write_text(content, encoding="utf-8", newline="")
    print(f"{content.count(chr(10)) - 1} bodies, {len(unresolved)} unresolved")
    for item in unresolved:
        print(item)


if __name__ == "__main__":
    main()
