modfied parameter


------------------------------------------------------------------------------
crazyswarm2/crazyflie/config/crazyflies.yaml

- set default crazyflie name as 1 for simplicity
cf231 -> cf1 (3rd line)

- set radio address correctly
uri: radio://0/80/2M/E7E7E7E7E7 -> uri: radio://0/80/2M/E7E7E7E7E7 (5th line)

- not using motion capture deck
enabled: true -> enabled: false (34th line)
enabled: true -> enabled: false (57th line)

------------------------------------------------------------------------------
