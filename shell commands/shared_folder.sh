#!/bin/bash

# Parce qu'on sait jamais
sudo apt remove open-vm-tools-desktop && sudo apt install open-vm-tools-desktop

# Après avoir ajouté un dossier partagé dans les paramètres de la VM
sudo nano /etc/fstab
# Add "vmhgfs-fuse     /mnt/hgfs       fuse    defaults,allow_other    0       0"
