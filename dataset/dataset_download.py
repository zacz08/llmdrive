"""
    @author: Cheng Zhang
    
"""

from huggingface_hub import snapshot_download
snapshot_download(
    repo_id='OpenDILabCommunity/LMDrive', 
    repo_type='dataset',
    ignore_patterns=[
        "*.md", 
        "*town01*",
        "*town02*",
        "*town03*",
        "*town04*",
        "*town05*",
        "*town06*",
        "*town10*",
        "*long*",
        "*short*"
        ], 
    cache_dir='Zachary/03_Data/python_Project/LMDrive/dataset',     # replace with your dictionary as needed
    local_dir_use_symlinks=True,
    etag_timeout=120,
    resume_download=True
    )