import express from "express";
import bodyParser from "body-parser";
import cors from "cors";
import path from "path";
import { fileURLToPath } from "url";
import { spawn } from "child_process";
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

// ROBOT STATUS
let robotStatus = {
  robot_number: "RB-01",
  status_key: "Available",
  status_text: "Idle",
  updated_at: new Date().toISOString()
};

const app = express();
app.use(cors());
app.use(bodyParser.json());

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
app.use(express.static(path.join(__dirname, "public")));

app.get("/", (_req, res) => {
  res.sendFile(path.join(__dirname, "public", "idle.html"));
});

app.get("/favicon.ico", (_req, res) => {
  res.status(204).end();
});

function normalizeInventoryLang(langRaw) {
  const v = String(langRaw || "en").trim().toLowerCase();
  if (!v || v === "en" || v === "en-us") return "en";
  if (v === "es" || v === "es-es" || v === "es-mx") return "es";
  if (v === "zh" || v === "zh-cn" || v === "zh-hans") return "zh-cn";
  return "en";
}

function translateWithDeepTranslator(texts, targetLang) {
  return new Promise((resolve, reject) => {
    const scriptPath = path.join(__dirname, "translate_inventory.py");
    const child = spawn("python3", [scriptPath, targetLang], { stdio: ["pipe", "pipe", "pipe"] });

    let stdout = "";
    let stderr = "";

    child.stdout.on("data", (chunk) => {
      stdout += chunk.toString();
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk.toString();
    });
    child.on("error", (err) => {
      reject(new Error(`translator spawn error: ${err.message}`));
    });
    child.on("close", (code) => {
      if (code !== 0) {
        reject(new Error(`translator exit ${code}: ${stderr.trim() || "unknown error"}`));
        return;
      }
      try {
        const parsed = JSON.parse(stdout);
        if (!Array.isArray(parsed)) {
          reject(new Error("translator output is not an array"));
          return;
        }
        resolve(parsed);
      } catch (err) {
        reject(new Error(`translator invalid JSON: ${err.message}`));
      }
    });

    child.stdin.write(JSON.stringify(texts));
    child.stdin.end();
  });
}

function memberIdToOrdinal(memberId) {
  const s = String(memberId || "").trim().toUpperCase();
  if (!/^[A-Z]{3}[0-9]{3}$/.test(s)) return null;

  const lettersPart = s.slice(0, 3);
  const digitsPart = Number(s.slice(3));

  const a = lettersPart.charCodeAt(0) - 65;
  const b = lettersPart.charCodeAt(1) - 65;
  const c = lettersPart.charCodeAt(2) - 65;
  if (a < 0 || a > 25 || b < 0 || b > 25 || c < 0 || c > 25) return null;

  const letterOrdinal = a * 26 * 26 + b * 26 + c;
  return letterOrdinal * 1000 + digitsPart;
}

function ordinalToMemberId(ordinal) {
  const n = Number(ordinal);
  if (!Number.isInteger(n) || n < 0) {
    throw new Error("Invalid member ordinal");
  }

  const letterOrdinal = Math.floor(n / 1000);
  const digitsPart = n % 1000;
  const maxLetterOrdinal = 26 * 26 * 26 - 1;
  if (letterOrdinal > maxLetterOrdinal) {
    throw new Error("Member ID space exhausted");
  }

  const a = Math.floor(letterOrdinal / (26 * 26));
  const b = Math.floor((letterOrdinal % (26 * 26)) / 26);
  const c = letterOrdinal % 26;

  const letters =
    String.fromCharCode(65 + a) +
    String.fromCharCode(65 + b) +
    String.fromCharCode(65 + c);

  return `${letters}${String(digitsPart).padStart(3, "0")}`;
}

const employeeSessions = new Map();
const EMPLOYEE_SESSION_TTL_MS = 12 * 60 * 60 * 1000; // 12 hours

function makeEmployeeToken(employeeId) {
  const rand = Math.random().toString(36).slice(2, 10);
  return `emp_${employeeId}_${Date.now()}_${rand}`;
}

function cleanExpiredEmployeeSessions() {
  const now = Date.now();
  for (const [token, session] of employeeSessions.entries()) {
    if (!session?.expiresAt || session.expiresAt <= now) {
      employeeSessions.delete(token);
    }
  }
}

function getEmployeeTokenFromRequest(req) {
  const auth = String(req.headers.authorization || "");
  if (auth.startsWith("Bearer ")) return auth.slice(7).trim();
  const fallback = String(req.headers["x-employee-token"] || "");
  return fallback.trim();
}

function requireEmployeeAuth(req, res, next) {
  cleanExpiredEmployeeSessions();
  const token = getEmployeeTokenFromRequest(req);
  if (!token) return res.status(401).json({ error: "Employee auth token required" });

  const session = employeeSessions.get(token);
  if (!session || session.expiresAt <= Date.now()) {
    employeeSessions.delete(token);
    return res.status(401).json({ error: "Employee session expired" });
  }

  req.employeeSession = session;
  next();
}

// CUSTOMER LOGIN (member only, no password)
app.post("/api/auth/login", async (req, res) => {
  try {
    const { member_ID } = req.body;
    if (!member_ID) {
      return res.status(400).json({ error: "member_ID is required" });
    }

    const result = await pool.query(
      `SELECT member_ID, first_name, last_name
       FROM customer
       WHERE member_ID = $1
       LIMIT 1`,
      [member_ID]
    );

    if (result.rows.length === 0) {
      return res.status(404).json({ error: "Member ID not found" });
    }

    const c = result.rows[0];
    res.json({
      access_level: "customer",
      member_ID: c.member_id ?? member_ID,
      first_name: c.first_name ?? "",
      last_name: c.last_name ?? "",
      guest: false,
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE LOGIN (password required, no guest mode)
app.post("/api/employee/auth/login", async (req, res) => {
  try {
    const employee_ID = String(req.body?.employee_ID || "").trim().toUpperCase();
    const password = String(req.body?.password || "");
    if (!employee_ID || !password) {
      return res.status(400).json({ error: "employee_ID and password are required" });
    }

    const result = await pool.query(
      `SELECT employee_ID, first_name, last_name, password
       FROM employee
       WHERE employee_ID = $1
       LIMIT 1`,
      [employee_ID]
    );

    if (result.rows.length === 0) {
      return res.status(404).json({ error: "Employee ID not found" });
    }

    const employeePassword = String(result.rows[0]?.password ?? "");
    if (!employeePassword || password !== employeePassword) {
      return res.status(401).json({ error: "Invalid password" });
    }

    const employee = result.rows[0];
    const token = makeEmployeeToken(employee_ID);
    employeeSessions.set(token, {
      token,
      employee_ID,
      first_name: employee.first_name ?? "",
      last_name: employee.last_name ?? "",
      createdAt: Date.now(),
      expiresAt: Date.now() + EMPLOYEE_SESSION_TTL_MS
    });

    res.json({
      ok: true,
      token,
      employee: {
        employee_ID: employee.employee_id ?? employee_ID,
        first_name: employee.first_name ?? "",
        last_name: employee.last_name ?? ""
      }
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE ACCOUNTS LIST
app.get("/api/employee/accounts", requireEmployeeAuth, async (_req, res) => {
  try {
    const result = await pool.query(
      `SELECT employee_ID, first_name, last_name
       FROM employee
       ORDER BY employee_ID`
    );
    res.json({
      items: result.rows.map((r) => ({
        employee_ID: r.employee_id ?? "",
        first_name: r.first_name ?? "",
        last_name: r.last_name ?? ""
      }))
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE ACCOUNT CREATE
app.post("/api/employee/accounts/create", requireEmployeeAuth, async (req, res) => {
  try {
    const employee_ID = String(req.body?.employee_ID || "").trim().toUpperCase();
    const first_name = String(req.body?.first_name || "").trim();
    const last_name = String(req.body?.last_name || "").trim();
    const password = String(req.body?.password || "");

    if (!employee_ID || !first_name || !last_name || !password) {
      return res.status(400).json({ error: "employee_ID, first_name, last_name, password are required" });
    }

    if (!/^[0-9]{3}[A-Z]{3}$/.test(employee_ID)) {
      return res.status(400).json({ error: "employee_ID must match format 000AAA" });
    }

    await pool.query(
      `INSERT INTO employee (employee_ID, first_name, last_name, password)
       VALUES ($1, $2, $3, $4)`,
      [employee_ID, first_name, last_name, password]
    );

    res.json({
      ok: true,
      employee_ID,
      first_name,
      last_name
    });
  } catch (e) {
    if (String(e.message || "").toLowerCase().includes("duplicate")) {
      return res.status(409).json({ error: "Employee ID already exists" });
    }
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE INVENTORY REPORT
app.get("/api/employee/inventory/report", requireEmployeeAuth, async (_req, res) => {
  try {
    const invResult = await pool.query(
      `SELECT product_id, product_name, category_name, stock, x, y, z
       FROM inventory
       ORDER BY product_name`
    );
    const items = invResult.rows.map((r) => ({
      product_id: r.product_id,
      product_name: r.product_name ?? "",
      category_name: r.category_name ?? "Uncategorized",
      stock: Number(r.stock ?? 0),
      x: r.x ?? null,
      y: r.y ?? null,
      z: r.z ?? null
    }));

    let unitsInStock = 0;
    let occupiedProducts = 0;
    let lowStockProducts = 0;
    const categoryMap = new Map();

    for (const it of items) {
      unitsInStock += it.stock;
      if (it.stock > 0) occupiedProducts += 1;
      if (it.stock <= 5) lowStockProducts += 1;

      const key = it.category_name || "Uncategorized";
      const agg = categoryMap.get(key) || { category: key, products: 0, units: 0 };
      agg.products += 1;
      agg.units += it.stock;
      categoryMap.set(key, agg);
    }

    const restockTodayResult = await pool.query(
      `SELECT COUNT(*)::int AS count
       FROM restock_id
       WHERE DATE(timestamp) = CURRENT_DATE`
    );
    const restockRequestsToday = Number(restockTodayResult.rows[0]?.count ?? 0);

    res.json({
      summary: {
        total_products: items.length,
        occupied_products: occupiedProducts,
        empty_products: items.length - occupiedProducts,
        units_in_stock: unitsInStock,
        low_stock_products: lowStockProducts,
        restock_requests_today: restockRequestsToday
      },
      category_summary: [...categoryMap.values()].sort((a, b) => a.category.localeCompare(b.category)),
      items
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE INVENTORY OPTIONS (for restock dropdown)
app.get("/api/employee/inventory/options", requireEmployeeAuth, async (_req, res) => {
  try {
    const result = await pool.query(
      `SELECT product_id, product_name, category_name, stock
       FROM inventory
       ORDER BY product_name`
    );
    res.json({
      items: result.rows.map((r) => ({
        product_id: r.product_id,
        product_name: r.product_name ?? "",
        category_name: r.category_name ?? "Uncategorized",
        stock: Number(r.stock ?? 0)
      }))
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// EMPLOYEE ROBOT STATUS (placeholder source for dashboard UI)
// EMPLOYEE ROBOT STATUS (real-time single robot)
app.get("/api/employee/robot/status", requireEmployeeAuth, async (_req, res) => {
  res.json({
    robots: [robotStatus]
  });
});

// EMPLOYEE RESTOCK SUBMIT (dashboard endpoint)
app.post("/api/employee/restock/submit", requireEmployeeAuth, async (req, res) => {
  let client = null;
  try {
    const employee_ID = req.employeeSession.employee_ID;
    const inputItems = Array.isArray(req.body?.items) ? req.body.items : [];
    if (inputItems.length === 0) {
      return res.status(400).json({ error: "items is required" });
    }

    const normalizedItems = inputItems.map((it) => ({
      product_id: Number(it.product_id),
      qty: Number(it.qty)
    })).filter((it) => Number.isInteger(it.product_id) && it.qty > 0);

    if (normalizedItems.length === 0) {
      return res.status(400).json({ error: "items must contain valid product_id and qty>0" });
    }

    const ids = normalizedItems.map((it) => it.product_id);
    const invResult = await pool.query(
      `SELECT product_id, product_name FROM inventory WHERE product_id = ANY($1::int[])`,
      [ids]
    );
    const namesById = new Map(invResult.rows.map((r) => [Number(r.product_id), r.product_name ?? ""]));

    const items = normalizedItems.map((it) => ({
      product_id: it.product_id,
      name: namesById.get(it.product_id) ?? "",
      qty: it.qty
    }));

    const restock_ID = `${employee_ID}R${Date.now()}`;

    client = await pool.connect();
    await client.query("BEGIN");
    await client.query(
      `INSERT INTO restock_id (restock_ID, employee_ID, items, timestamp, status)
       VALUES ($1, $2, $3, $4, 'RECEIVED')`,
      [restock_ID, employee_ID, JSON.stringify(items), new Date().toISOString()]
    );
    await client.query(
      `UPDATE employee SET restock_ID=$1 WHERE employee_ID=$2`,
      [restock_ID, employee_ID]
    );
    await client.query("COMMIT");

    robotStatus.status_key = "restock_in_progress";
    robotStatus.status_text = "Operate for Restock";
    robotStatus.updated_at = new Date().toISOString();

    res.json({ status: "RECEIVED", restock_ID });
  } catch (e) {
    if (client) {
      try { await client.query("ROLLBACK"); } catch (_) {}
    }
    res.status(500).json({ error: e.message });
  } finally {
    if (client) client.release();
  }
});

// CREATE CUSTOMER ACCOUNT
app.post("/api/account/create_customer", async (req, res) => {
  let client = null;
  try {
    const { first_name, last_name } = req.body;
    if (!first_name || !last_name) {
      return res.status(400).json({ error: "first_name and last_name are required" });
    }

    const firstNameText = String(first_name).trim();
    const lastNameText = String(last_name).trim();
    if (!firstNameText || !lastNameText) {
      return res.status(400).json({ error: "first_name and last_name are required" });
    }

    client = await pool.connect();
    await client.query("BEGIN");
    await client.query("LOCK TABLE customer IN EXCLUSIVE MODE");

    const existingRows = await client.query(`SELECT member_ID FROM customer`);
    let maxOrdinal = -1;
    for (const row of existingRows.rows) {
      const ord = memberIdToOrdinal(row.member_id ?? row.member_ID);
      if (ord !== null && ord > maxOrdinal) maxOrdinal = ord;
    }

    const nextOrdinal = maxOrdinal + 1;
    const memberIdText = ordinalToMemberId(nextOrdinal);

    await client.query(
      `INSERT INTO customer (member_ID, first_name, last_name)
       VALUES ($1, $2, $3)`,
      [memberIdText, firstNameText, lastNameText]
    );

    await client.query("COMMIT");

    res.json({
      ok: true,
      member_ID: memberIdText,
      first_name: firstNameText,
      last_name: lastNameText,
    });
  } catch (e) {
    if (client) {
      try {
        await client.query("ROLLBACK");
      } catch (_) {}
    }
    res.status(500).json({ error: e.message });
  } finally {
    if (client) client.release();
  }
});

// INVENTORY LIST (customer UI format)
app.get("/api/inventory/list", async (req, res) => {
  try {
    const lang = normalizeInventoryLang(req.query.lang);
    const result = await pool.query("SELECT * FROM inventory ORDER BY product_name");
    const items = result.rows.map((r) => ({
      id: r.product_id ?? r.id,
      name: r.product_name ?? r.name ?? "",
      category_id: r.category_id ?? null,
      category_name: r.category_name ?? "",
      price: r.price ?? 0,
      stock: Number(r.stock ?? 0),
      x: r.x ?? null,
      y: r.y ?? null,
      z: r.z ?? null,
    }));

    if (lang !== "en" && items.length > 0) {
      try {
        const englishNames = items.map((it) => String(it.name ?? ""));
        const englishCategories = items.map((it) => String(it.category_name ?? ""));

        const translatedNames = await translateWithDeepTranslator(englishNames, lang);
        const translatedCategories = await translateWithDeepTranslator(englishCategories, lang);

        if (
          translatedNames.length === items.length &&
          translatedCategories.length === items.length
        ) {
          for (let i = 0; i < items.length; i += 1) {
            items[i].name = String(translatedNames[i] ?? items[i].name);
            items[i].category_name = String(
              translatedCategories[i] ?? items[i].category_name
            );
          }
        }
      } catch (e) {
        console.warn(`inventory translation failed (lang=${lang}): ${e.message}`);
      }
    }

    res.json({ items });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// CUSTOMER ORDER (UI compatibility endpoint)
app.post("/api/order/customer", async (req, res) => {
  try {
    const member_ID = req.body.member_ID || req.body.id;
    const isGuest = Boolean(req.body.guest);
    const { items } = req.body;
    if (!Array.isArray(items) || items.length === 0) {
      return res.status(400).json({ error: "Invalid order format" });
    }
    if (!isGuest && !member_ID) {
      return res.status(400).json({ error: "member_ID is required for member orders" });
    }

    const requesterId = member_ID || "GUEST";
    const order_ID = requesterId + "O" + Date.now();
    const normalizedItems = items.map((it) => ({
      product_id: it.product_id,
      name: it.name,
      price: it.price,
      stock: it.stock,
      qty: it.qty,
      x: it.x ?? it.aisle ?? null,
      y: it.y ?? it.rack ?? null,
      z: it.z ?? it.shelf_level ?? null,
      requester_id: requesterId,
      guest: isGuest,
    }));
    const memberIdForOrder = isGuest ? null : member_ID;

    await pool.query(
      `INSERT INTO order_id (order_ID, member_ID, items, timestamp)
       VALUES ($1, $2, $3, $4)`,
      [order_ID, memberIdForOrder, JSON.stringify(normalizedItems), new Date().toISOString()]
    );

    if (!isGuest) {
      await pool.query(
        `UPDATE customer SET order_ID=$1 WHERE member_ID=$2`,
        [order_ID, member_ID]
      );
    }

    robotStatus.status_key = "customer_order_received";
    robotStatus.status_text = "Operate for Customer Order";
    robotStatus.updated_at = new Date().toISOString();


    res.json({ status: "RECEIVED", order_id: order_ID });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// ROBOT: fetch next pending order
app.get("/api/order/latest", async (_req, res) => {
  try {
    const result = await pool.query(
      `SELECT order_ID, member_ID, items, timestamp
       FROM order_id
       WHERE status = 'RECEIVED'
       ORDER BY timestamp ASC
       LIMIT 1`
    );

    if (result.rows.length === 0) {
      return res.status(204).end();
    }

    const row = result.rows[0];
    const rawItems = Array.isArray(row.items) ? row.items : [];
    const items = rawItems.map((it) => ({
      product_id: String(it.product_id ?? ""),
      name: it.name ?? "",
      price: Number(it.price ?? 0),
      stock: Number(it.stock ?? 0),
      qty: Number(it.qty ?? 0),
      x: Number(it.x ?? it.aisle ?? 0),
      y: Number(it.y ?? it.rack ?? 0),
      z: Number(it.z ?? it.shelf_level ?? 0),
    }));

    res.json({
      order_id: row.order_id,
      role: "customer",
      requester_id: row.member_id,
      items,
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// ROBOT: acknowledge an order is in progress
app.post("/api/order/ack", async (req, res) => {
  try {
    const order_ID = req.body?.order_id || req.body?.order_ID;
    if (!order_ID) {
      return res.status(400).json({ error: "order_id is required" });
    }

    await pool.query(
      `UPDATE order_id
       SET status = 'IN_PROGRESS'
       WHERE order_ID = $1 AND status = 'RECEIVED'`,
      [String(order_ID)]
    );

    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// ROBOT: decrement stock after successful pick/place
app.post("/api/inventory/decrement", async (req, res) => {
  try {
    const { product_id, qty } = req.body;
    const q = Number(qty || 0);
    if (!product_id || q <= 0) {
      return res.status(400).json({ error: "product_id and qty>0 are required" });
    }

    await pool.query(
      `UPDATE inventory
       SET stock = GREATEST(stock - $1, 0)
       WHERE product_id = $2`,
      [q, product_id]
    );

    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// ORDER STATUS (UI compatibility endpoint)
app.get("/api/order/status/:id", async (req, res) => {
  try {
    const result = await pool.query(
      `SELECT order_ID AS order_id, status, result, timestamp
       FROM order_id WHERE order_ID=$1`,
      [req.params.id]
    );

    if (result.rows.length === 0) {
      return res.status(404).json({ error: "Not found" });
    }

    const row = result.rows[0];
    res.json({
      order_id: row.order_id,
      status: row.status || "RECEIVED",
      result: row.result ?? null,
      timestamp: row.timestamp ?? null,
    });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

// ROBOT/UI completion endpoint
app.post("/api/order/complete", async (req, res) => {
  try {
    const order_ID = req.body.order_id || req.body.order_ID;
    const resultRaw = String(req.body.result || "");
    const ok = resultRaw.toUpperCase().includes("SUCCESS");
    const status = ok ? "DONE" : "FAILED";
    const result = ok ? "SUCCESS" : "FAILED";

    robotStatus.status_key = "Available";
    robotStatus.status_text = "Idle";
    robotStatus.updated_at = new Date().toISOString();

    if (!order_ID) {
      return res.status(400).json({ error: "order_id is required" });
    }

    await pool.query(
      `UPDATE order_id SET status=$1, result=$2 WHERE order_ID=$3`,
      [status, result, String(order_ID)]
    );

    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ error: e.message });
  }
});

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

app.post("/api/customer/ai/ask", async (req,res)=>{

  const question = String(req.body?.question || "")

  const inv = await pool.query(
    `SELECT product_name, category_name
     FROM inventory`
  )

  const prompt = `
You are a grocery store assistant.

Products available:
${JSON.stringify(inv.rows)}

Rules:
- only answer product questions
- recommend products
- do NOT reveal stock numbers
- do NOT discuss internal operations

Customer question:
${question}
`

  const r = await fetch("http://localhost:11434/api/generate",{
    method:"POST",
    headers:{ "Content-Type":"application/json"},
    body:JSON.stringify({
      model:"phi3",
      prompt,
      stream:false,
      options:{
        num_predict:150,
        temperature:0.3
      }
    }),
  })

  const data = await r.json()

  res.json({answer:data.response})
});

app.post("/api/employee/ai/ask", requireEmployeeAuth, async (req,res)=>{
  try{

    const question = String(req.body?.question || "")

    const inv = await pool.query(
      `SELECT product_name, category_name, stock
       FROM inventory`
    )

    const inventory = inv.rows

    const prompt = `
You are a grocery robot fleet manager assistant.

Inventory:
${JSON.stringify(inventory)}

Answer questions about:
- inventory
- stock
- categories
- restocking suggestions

Question:
${question}
`

    const r = await fetch("http://localhost:11434/api/generate",{
      method:"POST",
      headers:{ "Content-Type":"application/json"},
      body:JSON.stringify({
        model:"phi3",
        prompt,
        stream:false,
        options:{
          num_predict:150,
          temperature:0.3
        }
      })
    })

    const data = await r.json()

    res.json({answer:data.response})

  }catch(e){
    res.status(500).json({error:e.message})
  }
})

app.listen(3000, () =>
  console.log("Server running: http://localhost:3000")
);
