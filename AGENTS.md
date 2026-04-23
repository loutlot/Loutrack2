0. NEVER use local directory writing on .md files
1. When making implementation progress, update `/changelog.md` in the same change.
   - Add a concise bullet describing the user-visible behavior, workflow change, bug fix, or architectural progress.
   - Prefer concrete outcomes over process notes.
   - Keep entries in reverse chronological order, newest at the top.
   - Do not add duplicate or near-duplicate entries; update or consolidate the existing entry instead.
   - Mention affected areas or paths when useful, especially if files or directories moved.
   - Make them short.
2. When updating `/changelog.md`, verify that referenced file and directory paths still exist or clearly describe renamed/moved locations.
3. when adding or updating tests, do not leave overlapping temporary tests or stale duplicate test coverage; consolidate similar tests and remove leftovers in the same change
4. Pi / GUI startup and shutdown procedure lives at `docs/30_procedure/pi_gui_start_stop.md`
