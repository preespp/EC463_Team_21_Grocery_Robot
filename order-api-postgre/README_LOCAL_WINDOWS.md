# Local Run on Windows

## Prerequisites

- Node.js 20+
- Docker Desktop running

## 1. Start PostgreSQL

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

## 2. Start backend

```powershell
cd F:\EC463\EC463_Team_21_Grocery_Robot\order-api-postgre
npm install
npm run dev
```

Backend URL:

- `http://localhost:3000`

## 3. Start fleet UI

```powershell
cd F:\EC463\EC463_Team_21_Grocery_Robot\order-api-postgre\fleet-manager
npm install
npm run dev
```

Frontend URL:

- `http://localhost:5174`

## Notes

- The map pages require employee login because the API uses bearer auth.
- The semantic map is read from `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`.
- The base raster is read from `Maps/testmapMain.pgm`.
