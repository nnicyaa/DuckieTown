@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0apply_kvati_target.ps1" %*
