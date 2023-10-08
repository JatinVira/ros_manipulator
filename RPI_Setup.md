# README: Installing Ubuntu 20.04 LTS on Raspberry Pi

This guide will walk you through the process of installing Ubuntu 20.04 LTS on your Raspberry Pi using the Raspberry Pi Imager software.

## Prerequisites

- A Raspberry Pi 4
- A microSD card (16GB or larger)
- A computer with an SD card reader

## Steps

### 1. Download and Install Raspberry Pi Imager

1. Install Raspberry Pi Imager on Ubuntu using the following command:

    ```bash
    sudo snap install rpi-imager
    ```

### 2. Select Ubuntu 20.04 LTS in Raspberry Pi Imager

1. Open Raspberry Pi Imager and select "Choose OS".

2. Under the "Other general purpose" tab, select "Ubuntu" and choose the "Ubuntu Server 20.04.5 LTS (64 bit)" option.

3. Select the microSD card under "Choose SD Card".

### 3. Enable SSH

1. Click on "Toggle on" next to "SSH" to enable SSH for remote access.

### 4. Write Ubuntu to the microSD Card

1. Click on "Write" and confirm any prompts.

2. Wait for the writing process to complete. This may take a few minutes.

### 5. Booting Up

1. Remove the microSD card from your computer and insert it into your Raspberry Pi.

2. Connect peripherals (keyboard, mouse, monitor) and power up your Raspberry Pi.

3. Follow the on-screen instructions to complete the Ubuntu setup process.

### 6. Initial Login

1. Once the setup process is complete, you will be prompted to log in. Use the default username `ubuntu` and the password provided during the setup.

### 7. Additional Configuration (Optional)

- If needed, you can further configure your Raspberry Pi and install additional software or packages.

---
