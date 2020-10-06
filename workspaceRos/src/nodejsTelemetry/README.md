# Sailboat telemetry

This project is a [Node.js](https://nodejs.org/en/) application designed for our MicroTransat sailboat.
It can be configured to be ran either on the boat's Rasberry Pi or on pc a with a socket connection.

## Installation

The dependencies are [Node.js](https://nodejs.org/en/) and [MongoDB](https://www.mongodb.com/fr).
You can check [this website](https://nodejs.org/en/download/) to install Node.js and [this one](https://docs.mongodb.com/guides/server/install/) to install MongoDB.

After having cloned this repository, in order to install all of the Node.js packages required, you should run
```sh
npm install
```


## Configuration

All of the settings are in the `.env` file. 
Every one of them should be checked after installation.

## Usage

Start the program :

```sh
node start.js
```

The portal should appear on the Raspberry Pi's IP address, port 3000.