# Robotics engineering

Two robot projects share one Git repository:

- `spidar300/` — hexapod model and Isaac Lab workflow.
- `arm100/` — SO-100/SO-101 CAD and simulation assets.

Store source code and robot models here. Store datasets and trained policies in
Hugging Face Hub.

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://hf.co/cli/install.ps1 | iex"
hf auth login
hf upload 80n3yB4dg3r/SPIDAR300 . --repo-type dataset
```

See `spidar300/HUB.md` for the SPIDAR dataset and model workflow.
