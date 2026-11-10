# Home tag: measuring optical-flow drift (`tags.home_tag_id`)

PX4 navigates on optical flow alone, so its position slowly drifts and it
can't tell. A tag that never moves shows the drift: PX4 places every tag using
its own position, so the home tag *appears* to move by exactly PX4's error.

```
hover before the sweep   home tag seen at T0     (no drift yet)
sweep                    PX4 drifts by e, unseen
back over "home"         home tag seen at T1     ->  e = T1 - T0
move by e, look again, land when the tag is under the drone
```

## Setup

1. Print one more tag of the **same family and size** as the search tags
   (tag16h5, `tags.tag_size_m`), with an ID that is **not** a search tag,
   e.g. 29.
2. Tape it flat on the floor at the takeoff spot and put the drone on it.
   The camera sees it once the drone hovers.
3. In `mission/config.yaml`:

   ```yaml
   tags:
     home_tag_id: 29
   ```

No firmware or PX4 changes.

## What happens

| When | What | Event in `events.csv` |
|---|---|---|
| Hover, before the sweep | home tag position recorded | `HOME_TAG_REF` (or `HOME_TAG_MISSING`: no correction this flight) |
| Back home, settled | looks for the tag for up to `home_tag_wait_s` | |
| Tag seen | drift worked out | `HOME_TAG_DRIFT` |
| Drift bigger than `home_tag_tol_m` (5 cm) | flies by that much, settles, looks again | `HOME_TAG_MOVE` |
| Tag under the drone, or `home_tag_max_moves` used | lands | `LAND_CMD ... on the home tag within N cm` |
| Tag not seen | lands where it is | `HOME_TAG_NOT_SEEN` |

The home tag is not a search target: it isn't on the map, in the report, or in
`expected_count`.

## Correcting the search tags

With the drift measured, every search tag is corrected too. The drift is
assumed to grow **in proportion to the distance flown**, from 0 at the start of
the sweep to the measured value at home. Optical-flow error mostly grows with
distance, not time. So a tag seen early in the sweep gets a small correction
and a late one nearly the full drift.

- The report and `tags_final.csv` `x`, `y` are the corrected positions.
- `x_raw`, `y_raw` are what PX4 saw.

This is only as good as that assumption. A steady drift (flow sensor slightly
rotated, wrong scale) is corrected well. A sudden jump halfway through is not.

## Limits

- **The drift must be smaller than the camera's view** at home (about 0.3 m
  either way at 0.5 m height), or the tag isn't under the camera when the drone
  comes back. It then lands where it is and logs `HOME_TAG_NOT_SEEN`. Flying
  slower (0.4 m/s) keeps the drift smaller.
- **Only position drift.** The heading must be right, which it is on IMU0. A
  heading error would also rotate the path, which a single offset can't fully
  describe.
- A drift over `home_tag_max_drift_m` (1 m) is taken as a wrong sighting and ignored.

UWB (`uwb.ground_frame`) corrects the position continuously, during the whole
flight. The home tag only measures it at the end.

## Settings (`tags:` in config.yaml)

| Setting | Default | Meaning |
|---|---|---|
| `home_tag_id` | -1 | the home tag's ID; -1 = off |
| `home_tag_min_sightings` | 5 | sightings for each fix |
| `home_tag_wait_s` | 8 | how long to look back home before landing anyway |
| `home_tag_tol_m` | 0.05 | land once a correction would move less than this |
| `home_tag_max_moves` | 3 | correction moves before landing anyway |
| `home_tag_max_drift_m` | 1.0 | bigger "drift" = wrong sighting, ignored |
| `home_tag_correct_tags` | true | also correct the search tags |
