# How to run this server

## Local Run on Linux

This setup runs `order-api-postgre` directly on Linux with a native PostgreSQL service + Gemini API. Docker is not required.

### Prerequisites

- Ubuntu/Debian-style Linux shell commands are used below
- Node.js 20+
- npm 10+
- PostgreSQL 14+
- Python 3 if you want inventory translation to work
- `psql` available in your shell
- Gemini API

### 1. Install PostgreSQL + Gemini

```bash
sudo apt update
sudo apt install -y postgresql postgresql-contrib
sudo systemctl enable --now postgresql
```

```bash
# Obtain a Google AI API key from https://makersuite.google.com/app/apikey
# Create the config directory and gemini.json file
mkdir -p config
echo '{
  "apiKey": "...",
  "model": "gemini-2.0-flash"
}' > config/gemini.json
```

### 2. Create the database and role

The backend defaults to these connection values:

- host: `localhost`
- port: `5432`
- db: `grocery_inventory`
- user: `grocerybot`
- password: `team21`

Create them with:

```bash
sudo -u postgres psql
```

Inside `psql`:

```sql
CREATE ROLE grocerybot WITH LOGIN PASSWORD 'team21';
CREATE DATABASE grocery_inventory OWNER grocerybot;
\q
```

If the role or database already exists, skip the matching command.

### 3. Load schema and seed data

From the repo root:

```bash
cd order-api-postgre
PGPASSWORD=team21 psql -h localhost -U grocerybot -d grocery_inventory -f db/init/001_schema.sql
PGPASSWORD=team21 psql -h localhost -U grocerybot -d grocery_inventory -f db/init/002_seed.sql
```

Seed employee login:

- `000AAA` / `team21`

### 4. Start the backend

From `order-api-postgre`:

```bash
npm install
python3 -m pip install deep-translator
export PGHOST=localhost
export PGPORT=5432
export PGDATABASE=grocery_inventory
export PGUSER=grocerybot
export PGPASSWORD=team21
npm run dev
```

Backend URL:

- `http://localhost:3000`

Notes:

- You can omit the `export` lines if you want to use the backend defaults.
- `PORT` can also be overridden, for example `export PORT=3001`.
- `node server.js` also works if you do not want file-watch mode.
- You do not need `npm init -y` here because `package.json` is already committed.

### 5. Start the fleet UI

Open a second terminal:

```bash
cd order-api-postgre/fleet-manager
npm install
npm run dev -- --host 0.0.0.0
```

Frontend URL:

- `http://localhost:5174`

Notes:

- You do not need `npm init -y` in `fleet-manager` either; the Vite/Vue project files are already committed.

### 6. Optional ROS2 BT executor

If you want the full robot stack to consume orders/restock tasks after the API and UI are up,
open a third terminal and run the ROS2 task manager node.

Example:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source workspace/install/setup.bash
ros2 run robot_task_manager bt_executor
```

This is only meaningful if your ROS2 workspace has already been built and the required Nav2 /
manipulation nodes are running.

### 7. Optional database checks

Connect to PostgreSQL:

```bash
PGPASSWORD=team21 psql -h localhost -U grocerybot -d grocery_inventory
```

Useful checks:

```sql
\dt
SELECT * FROM employee;
SELECT * FROM inventory;
SELECT * FROM semantic_map_versions ORDER BY version_seq DESC;
```

### Embedded ROS Web GUI Attribution

`SlamMapView` in the fleet UI uses an adapted copy of `ros_web_gui_app`.

- Original upstream repository: https://github.com/chengyangkj/ros_web_gui_app
- Fork used by this repository: https://github.com/StarLionJiang/ros_web_gui_app
- Local adaptation in this repository: English localization and embedded integration for the Grocery Robot web dashboard
- Vendored source location: `third_party/ros_web_gui_app`

License summary for the embedded ROS Web GUI component:

- License: `CC BY-NC-SA 4.0`
- Allowed: study, research, and personal use
- Allowed: modification and redistribution, provided the original project attribution is preserved
- Not allowed: commercial use
- Required: derivative works must remain under the same license and include a link to the original project

For full license terms, see `third_party/ros_web_gui_app/LICENSE`.

Acknowledgements:

- Special thanks to the Lichtblick project for the open-source visualization and message-processing ecosystem, especially `@lichtblick/rosmsg` and `@lichtblick/rosmsg-serialization`.
- Thanks to all contributors and users of the original project and this adapted version.

Original upstream markdown preserved in the vendor tree:

- `third_party/ros_web_gui_app/README.md`

### Notes

- The map pages require employee login because the API uses bearer auth.
- The semantic map is read from `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`.
- The base raster is read from `Maps/testmapMain.pgm`.
- `docker-compose.yml` is optional local tooling only; this README does not depend on it.
- `firebase-admin` is not used by the current `order-api-postgre` code, so it is not required in setup.

## Local Run on Windows

### Prerequisites

- Node.js 20+
- Docker Desktop running

### 1. Start PostgreSQL

```powershell
cd F:\EC463\EC463_Team_21_Grocery_Robot\order-api-postgre
docker compose up -d
```

Database credentials:

- host: `localhost`
- port: `5432`
- db: `grocery_inventory`
- user: `grocerybot`
- password: `team21`

Seed employee login:

- `000AAA` / `team21`

### 2. Start backend

```powershell
cd F:\EC463\EC463_Team_21_Grocery_Robot\order-api-postgre
npm install
npm run dev
```

Backend URL:

- `http://localhost:3000`

### 3. Start fleet UI

```powershell
cd F:\EC463\EC463_Team_21_Grocery_Robot\order-api-postgre\fleet-manager
npm install
npm run dev
```

Frontend URL:

- `http://localhost:5174`

### Notes

- The map pages require employee login because the API uses bearer auth.
- The semantic map is read from `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`.
- The base raster is read from `Maps/testmapMain.pgm`.
