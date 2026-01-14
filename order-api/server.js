import express from "express";
import bodyParser from "body-parser";
import cors from "cors";

const app = express();
app.use(cors());
app.use(bodyParser.json());

let latestOrder = null;
let orderAvailable = false;

function OrderPayload(req, res, role) {
  const { id, items } = req.body;

  if (!id) {
    return res.status(400).json({ error: "Missing id" });
  }

  if (!Array.isArray(items) || items.length === 0) {
    return res.status(400).json({ error: "Empty order" });
  }

  for (const item of items) {
    const required = [
      "aisle",
      "name",
      "price",
      "rack",
      "shelf_level",
      "stock",
      "qty"
    ];

    for (const key of required) {
      if (!(key in item)) {
        return res.status(400).json({
          error: `Missing field '${key}' in item`
        });
      }
    }
  }

  latestOrder = {
    order_id: Date.now(),
    requester_id: id,
    role,
    items,
    timestamp: new Date().toISOString()
  };

  orderAvailable = true;

  console.log(`New ${role.toUpperCase()} order received:`);
  console.dir(latestOrder, { depth: null });

  return res.json({
    status: "RECEIVED",
    order_id: latestOrder.order_id
  });
}

// All API
app.post("/api/order/customer", (req, res) =>
  OrderPayload(req, res, "customer")
);

app.post("/api/order/employee", (req, res) =>
  OrderPayload(req, res, "employee")
);

app.get("/api/order/latest", (req, res) => {
  if (!orderAvailable) {
    return res.status(204).send();
  }
  res.json(latestOrder);
});

app.post("/api/order/ack", (req, res) => {
  console.log("Order acknowledged by ROS");
  orderAvailable = false;
  res.json({ status: "ACKED" });
});

app.listen(3000, () => {
  console.log("🚀 Node.js Order API running on http://localhost:3000");
});
