FROM potato-navigation-noetic:latest
RUN apt-get update && apt-get install -y --no-install-recommends python3-serial && rm -rf /var/lib/apt/lists/*
WORKDIR /rover_ws
