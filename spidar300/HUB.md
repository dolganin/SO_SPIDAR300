# Hugging Face Hub workflow

This repository contains only robot sources and code. Demonstrations, datasets,
training outputs and model weights live in Hugging Face Hub.

Install and authenticate once in PowerShell:

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://hf.co/cli/install.ps1 | iex"
hf auth login
```

The canonical dataset repository is `80n3yB4dg3r/SPIDAR300`.

```powershell
# Publish a prepared dataset directory.
hf upload 80n3yB4dg3r/SPIDAR300 .\data\spidar300 --repo-type dataset

# Train straight from the Hub dataset; weights remain outside Git.
python sim/scripts/train_lerobot.py --dataset-repo-id 80n3yB4dg3r/SPIDAR300

# Publish a completed run as a model repository.
hf upload 80n3yB4dg3r/SPIDAR300-ACT .\models\spidar300-act --repo-type model

# Evaluate a model fetched from the Hub cache automatically.
python sim/scripts/eval.py --model-repo-id 80n3yB4dg3r/SPIDAR300-ACT --headless
```
