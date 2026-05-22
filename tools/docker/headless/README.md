# Headless docer containers

## Usage

Build:

```sh
docker compose build
```

Start:

```sh
docker compose up -d

```

Stop:


```sh
docker compose down
```

Cleanup:

```sh
docker compose down -v --rmi all
docker builder prune --all --force
docker system prune --all --force
```
