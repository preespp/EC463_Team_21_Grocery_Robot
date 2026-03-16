CREATE TABLE IF NOT EXISTS customer (
  member_id VARCHAR(16) PRIMARY KEY,
  first_name VARCHAR(64) NOT NULL,
  last_name VARCHAR(64) NOT NULL,
  order_id VARCHAR(32)
);

CREATE TABLE IF NOT EXISTS employee (
  employee_id VARCHAR(16) PRIMARY KEY,
  first_name VARCHAR(64) NOT NULL,
  last_name VARCHAR(64) NOT NULL,
  password VARCHAR(128) NOT NULL,
  restock_id VARCHAR(32)
);

CREATE TABLE IF NOT EXISTS inventory (
  product_id SERIAL PRIMARY KEY,
  product_name VARCHAR(128) NOT NULL,
  category_id INTEGER NOT NULL,
  category_name VARCHAR(64) NOT NULL,
  price NUMERIC(10, 2) NOT NULL DEFAULT 0,
  stock INTEGER NOT NULL DEFAULT 0,
  x DOUBLE PRECISION,
  y DOUBLE PRECISION,
  z INTEGER
);

CREATE TABLE IF NOT EXISTS order_id (
  order_id VARCHAR(32) PRIMARY KEY,
  member_id VARCHAR(16),
  items JSONB NOT NULL DEFAULT '[]'::jsonb,
  timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  status VARCHAR(32) NOT NULL DEFAULT 'RECEIVED',
  result VARCHAR(32)
);

CREATE TABLE IF NOT EXISTS restock_id (
  restock_id VARCHAR(32) PRIMARY KEY,
  employee_id VARCHAR(16) NOT NULL,
  items JSONB NOT NULL DEFAULT '[]'::jsonb,
  timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  status VARCHAR(32) NOT NULL DEFAULT 'RECEIVED',
  result VARCHAR(32)
);

CREATE TABLE IF NOT EXISTS semantic_map_versions (
  id BIGSERIAL PRIMARY KEY,
  map_name VARCHAR(128) NOT NULL,
  semantic_id VARCHAR(128) NOT NULL,
  version_seq INTEGER NOT NULL,
  saved_by_employee_id VARCHAR(16),
  saved_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  change_summary TEXT NOT NULL DEFAULT '',
  source_yaml_path TEXT NOT NULL,
  yaml_text TEXT NOT NULL,
  semantic_json JSONB NOT NULL DEFAULT '{}'::jsonb,
  CONSTRAINT semantic_map_versions_unique_version UNIQUE (map_name, version_seq)
);

CREATE INDEX IF NOT EXISTS idx_inventory_product_name ON inventory (product_name);
CREATE INDEX IF NOT EXISTS idx_order_status_timestamp ON order_id (status, timestamp);
CREATE INDEX IF NOT EXISTS idx_restock_status_timestamp ON restock_id (status, timestamp);
CREATE INDEX IF NOT EXISTS idx_semantic_map_versions_lookup
  ON semantic_map_versions (map_name, semantic_id, version_seq DESC);
