#!/bin/bash
nrfjprog --family  nRF51 -e
nrfjprog --family  nRF51 --program _build/nrf51422_xxac.hex
nrfjprog --family  nRF51 -r