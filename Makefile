container_name ?= mocap-dev
image_name ?= mocap:base

start:
	docker container start $(container_name)

run:
	docker container run -it \
		-v $(pwd)/deps:/home/catkin_ws/src \
		--name $(container_name) \
		--network="host" \
		$(image_name)

build:
	docker build -t $(image_name) .

attach:
	docker container attach $(container_name)

ssh:
	docker exec -it $(container_name) bash