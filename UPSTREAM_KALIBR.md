# Upstream Kalibr snapshot

- Repository: `https://github.com/ethz-asl/kalibr`
- Commit: `1f60227442d25e36365ef5f72cd80b9666d73467`
- Local source used for the snapshot: `/home/czh/kalibr_workspace/src/kalibr`
- Snapshot directory: `upstream/kalibr`
- License: BSD; the upstream `LICENSE` file is preserved in the snapshot.

Run `scripts/verify_upstream.py` to compare every file against
`upstream/kalibr.sha256`. The snapshot must not be edited. Build adapters,
compatibility modules, and any explicitly reviewed patches belong outside it.

The snapshot contains upstream's own `.gitignore` files, which match some
tracked upstream files (`*.jpg`, `*.orig`, and the generated-looking
`CMakeLists.txt` files of the `catkin_simple` test scenarios). When refreshing
the snapshot, stage it with `git add -f upstream/kalibr` so these files are
not silently dropped; otherwise `scripts/verify_upstream.py` fails on a fresh
clone.
