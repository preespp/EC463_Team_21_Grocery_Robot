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

app.listen(3000, () =>
  console.log("Server running: http://localhost:3000")
);
