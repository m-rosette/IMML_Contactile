# IMML_Contactile

Installation instructions:
1. Clone the repo
```
git clone [this repo]
cd path/to/repo
```

2. Build the docker image:
```
docker build - < .docker/Dockerfile -t osrf/ros:humble-desktop-full --no-cache
```

3. Run the docker container:
```
./run_docker.bash
```