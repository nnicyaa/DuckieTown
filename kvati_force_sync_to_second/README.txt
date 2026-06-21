This package force-syncs the files that differ between KvatiTown(3).zip and KvatiTown(4).zip.
It is safer than git apply for your case because your local files no longer match the exact patch base.

Usage from your KvatiTown project root after extracting this zip into the project root:

powershell -ExecutionPolicy Bypass -File .\kvati_force_sync_to_second\apply_kvati_target.ps1

git diff --stat
python launch.py --sim --task convoying

The script backs up replaced/deleted files to kvati_backup_before_target_YYYYMMDD_HHMMSS.
