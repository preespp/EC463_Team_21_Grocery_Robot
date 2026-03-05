import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import path from "path";
import { fileURLToPath } from "url";
import pkg from "pg";

const { Pool } = pkg;

// PostgreSQL connection
const pool = new Pool({
  user: "grocerybot",
  host: "localhost",
  database: "grocery_inventory",
  password: "team21",
  port: 5432,
});

const app = express();
app.use(cors());
app.use(bodyParser.json());

// CUSTOMER ORDER
app.post("/order/customer/new", async (req, res) => {
  try {
    const { member_ID, items } = req.body;
    if (!member_ID || !Array.isArray(items) || items.length === 0)
      return res.status(400).json({ error: "Invalid order format" });

    const order_ID = member_ID + "O" + Date.now();

    await pool.query(
      `INSERT INTO order_id (order_ID, member_ID, items, timestamp)
       VALUES ($1, $2, $3, $4)`,
      [order_ID, member_ID, JSON.stringify(items), new Date().toISOString()]
    );

    await pool.query(
      `UPDATE customer SET order_ID=$1 WHERE member_ID=$2`,
      [order_ID, member_ID]
    );

    res.json({ status: "RECEIVED", order_ID });

  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE RESTOCK 
app.post("/order/employee/new", async (req, res) => {
  try {
    const { employee_ID, items } = req.body;
    if (!employee_ID || !Array.isArray(items))
      return res.status(400).json({ error: "Invalid restock format" });

    const restock_ID = employee_ID + "R" + Date.now();

    await pool.query(
      `INSERT INTO restock_id (restock_ID, employee_ID, items, timestamp)
       VALUES ($1, $2, $3, $4)`,
      [restock_ID, employee_ID, JSON.stringify(items), new Date().toISOString()]
    );

    await pool.query(
      `UPDATE employee SET restock_ID=$1 WHERE employee_ID=$2`,
      [restock_ID, employee_ID]
    );

    res.json({ status: "RECEIVED", restock_ID });

  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});


// INVENTORY LIST
app.get("/inventory/list/all", async (req, res) => {
  const result = await pool.query("SELECT * FROM inventory ORDER BY product_name");
  res.json({ items: result.rows });
});

// INVENTORY LIST
app.get("/inventory/list/category", async (req, res) => {
  const { category_id } = req.body;
  const result = await pool.query("SELECT * FROM inventory WHERE category_id=$1 ORDER BY product_name", 
    [req.query.category_id]);
  res.json({ items: result.rows });
});


// ADD INVENTORY
app.post("/inventory/add", async (req, res) => {
  const { product_name, category_id, category_name, stock, x, y, z } = req.body;

  await pool.query(
    `INSERT INTO inventory
     (product_name, category_id, category_name, stock, x, y, z)
     VALUES ($1,$2,$3,$4,$5,$6,$7)`,
    [product_name, category_id, category_name, stock, x, y, z]
  );

  res.json({ ok: true });
});

// UPDATE INVENTORY
app.post("/inventory/update", async (req, res) => {
  const { product_id, stock, x, y, z } = req.body;

  await pool.query(
    `UPDATE inventory
     SET stock=$1, x=$2, y=$3, z=$4
     WHERE product_id=$5`,
    [stock, x, y, z, product_id]
  );

  res.json({ ok: true });
});

// DELETE INVENTORY
app.post("/inventory/delete", async (req, res) => {
  const { product_id } = req.body;
  await pool.query("DELETE FROM inventory WHERE product_id=$1", [product_id]);
  res.json({ ok: true });
});

// ORDER STATUS
app.get("/order/status/:id", async (req, res) => {
  const result = await pool.query(
    `SELECT order_ID, status, result, timestamp
     FROM order_id WHERE order_ID=$1`,
    [req.params.id]
  );
  if (result.rows.length === 0)
    return res.status(404).json({ error: "Not found" });

  res.json(result.rows[0]);
});

// COMPLETE ORDER
app.post("/order/complete", async (req, res) => {
  const { order_ID, result } = req.body;

  const status = result === "SUCCESS" ? "DONE" : "FAILED";

  await pool.query(
    `UPDATE order_id SET status=$1, result=$2 WHERE order_ID=$3`,
    [status, result, order_ID]
  );

  res.json({ ok: true });
});

app.listen(3000, () =>
  console.log("Server running: http://localhost:3000")
);