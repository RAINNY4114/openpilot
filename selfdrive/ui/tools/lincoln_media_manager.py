#!/usr/bin/env python3
import argparse
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

LOCAL_ROOT = Path("/data/media/0/realdata")
DEFAULT_DEST = "NAS@192.168.50.200:/volume1/openpilot"
DEFAULT_PORT = "22"
DEFAULT_KEY = ""


def put_status(params: Params, message: str) -> None:
  cloudlog.info(message)
  params.put("LincolnNASLastResult", message)


def ensure_defaults(params: Params) -> tuple[str, str, str]:
  dest = params.get("NasSshDest") or DEFAULT_DEST
  port = params.get("NasSshPort") or DEFAULT_PORT
  key = params.get("NasSshKey")
  if key is None:
    key = DEFAULT_KEY

  params.put("NasSshDest", dest)
  params.put("NasSshPort", port)
  params.put("NasSshKey", key)
  return dest, port, key


def parse_dest(dest: str) -> tuple[str, str, str]:
  if ":" in dest:
    target, remote_path = dest.split(":", 1)
  else:
    target, remote_path = dest, ""
  if "@" in target:
    user, host = target.split("@", 1)
  else:
    user, host = "NAS", target
  remote_path = remote_path or "/"
  return user or "NAS", host or "unknown", remote_path


def build_ssh_base(user: str, host: str, port: str, key: str) -> list[str]:
  base = ["ssh", "-p", port, "-o", "StrictHostKeyChecking=no", "-o", "BatchMode=yes"]
  if key:
    base += ["-i", key]
  base.append(f"{user}@{host}")
  return base


def build_scp_base(port: str, key: str) -> list[str]:
  base = ["scp", "-r", "-P", port, "-o", "StrictHostKeyChecking=no", "-o", "BatchMode=yes"]
  if key:
    base += ["-i", key]
  return base


def run_upload(params: Params) -> None:
  dest, port, key = ensure_defaults(params)
  user, host, remote_path = parse_dest(dest)
  ssh_base = build_ssh_base(user, host, port, key)
  scp_base = build_scp_base(port, key)
  remote_root = f"{user}@{host}:{remote_path.rstrip('/')}"

  if not LOCAL_ROOT.exists():
    put_status(params, "No local recordings directory found.")
    return

  entries = sorted([p for p in LOCAL_ROOT.iterdir()])
  if not entries:
    put_status(params, "No local recordings to upload.")
    return

  mkdir_cmd = ssh_base + [f"mkdir -p {shlex.quote(remote_path)}"]
  result = subprocess.run(mkdir_cmd, capture_output=True, text=True)
  if result.returncode != 0:
    put_status(params, f"Failed to prepare NAS directory: {result.stderr.strip() or result.stdout.strip()}")
    return

  to_upload = []
  skipped = 0
  for entry in entries:
    remote_entry = f"{remote_path.rstrip('/')}/{entry.name}"
    check_cmd = ssh_base + [f"test -e {shlex.quote(remote_entry)}"]
    exists = subprocess.run(check_cmd)
    if exists.returncode == 0:
      skipped += 1
      continue
    to_upload.append(entry)

  if not to_upload:
    put_status(params, f"No new recordings to upload. Skipped {skipped} existing item(s).")
    return

  total = len(to_upload)
  uploaded = 0
  for idx, entry in enumerate(to_upload, 1):
    put_status(params, f"Upload progress: {idx}/{total}")
    scp_cmd = scp_base + [str(entry), f"{remote_root}/"]
    result = subprocess.run(scp_cmd, capture_output=True, text=True)
    if result.returncode != 0:
      put_status(params, f"Upload failed on {entry.name}: {result.stderr.strip() or result.stdout.strip()}")
      return
    uploaded += 1

  put_status(params, f"Upload finished: uploaded {uploaded} item(s), skipped {skipped}.")


def run_delete(params: Params) -> None:
  if not LOCAL_ROOT.exists():
    put_status(params, "No local recordings directory found.")
    return

  count = 0
  for entry in LOCAL_ROOT.iterdir():
    try:
      if entry.is_dir():
        shutil.rmtree(entry)
      else:
        entry.unlink()
      count += 1
    except Exception as e:
      put_status(params, f"Failed to delete {entry.name}: {e}")
      return

  put_status(params, f"Local recordings deleted (processed {count} item(s)).")


def main():
  parser = argparse.ArgumentParser(description="Lincoln NAS recording manager")
  parser.add_argument("--upload", action="store_true", help="Upload recordings to NAS")
  parser.add_argument("--delete", action="store_true", help="Delete local recordings")
  args = parser.parse_args()

  if args.upload == args.delete:
    parser.error("Select exactly one action (--upload or --delete)")

  params = Params()
  try:
    if args.upload:
      run_upload(params)
    else:
      run_delete(params)
  except Exception as e:
    put_status(params, f"NAS command failed: {e}")
    raise


if __name__ == "__main__":
  sys.exit(main())
