import psycopg2
from psycopg2.extras import Json


# Fill this template with the data you want to insert.
# Keep foreign-key order in mind:
# - order_id.member_ID must already exist in customer
# - restock_id.employee_ID must already exist in employee
DATA_TO_INSERT = {
    "customer": [
        {"member_ID": "AAA000", "first_name": "Pree", "last_name": "Simphliphan", "order_ID": None},
        {"member_ID": "AAA001", "first_name": "Lion", "last_name": "Jiang", "order_ID": None},
        {"member_ID": "AAA002", "first_name": "Feng", "last_name": "Tai", "order_ID": None},
        {"member_ID": "AAA003", "first_name": "Andy", "last_name": "Nguyen", "order_ID": None},
        {"member_ID": "AAA004", "first_name": "Darren", "last_name": "Sajino", "order_ID": None},
        {"member_ID": "AAA005", "first_name": "Bernie", "last_name": "Su", "order_ID": None},
    ],
    "employee": [
        {"employee_ID": "000AAA", "first_name": "Pree", "last_name": "Simphliphan", "restock_ID": None},
        {"employee_ID": "001AAA", "first_name": "Lion", "last_name": "Jiang", "restock_ID": None},
        {"employee_ID": "002AAA", "first_name": "Feng", "last_name": "Tai", "restock_ID": None},
        {"employee_ID": "003AAA", "first_name": "Andy", "last_name": "Nguyen", "restock_ID": None},
        {"employee_ID": "004AAA", "first_name": "Darren", "last_name": "Sajino", "restock_ID": None},
        {"employee_ID": "005AAA", "first_name": "Bernie", "last_name": "Su", "restock_ID": None},
    ],
    "inventory": [
        {"product_name": "Milk", "category_id": 1, "category_name": "Drinks", "price": 3.99, "stock": 20, "x": 1.0, "y": 2.0, "z": 1,},
        {"product_name": "Water", "category_id": 1, "category_name": "Drinks", "price": 1.49, "stock": 100, "x": 1.0, "y": 2.0, "z": 2,},
        {"product_name": "Apple", "category_id": 2, "category_name": "Fruits", "price": 0.99, "stock": 10, "x": 5.0, "y": 7.0, "z": 2,},
        {"product_name": "Orange", "category_id": 2, "category_name": "Fruits", "price": 1.19, "stock": 30, "x": 5.0, "y": 7.0, "z": 1,},
        {"product_name": "Rays", "category_id": 3, "category_name": "Snacks", "price": 4.29, "stock": 201, "x": 10.0, "y": 5.0, "z": 1,},
        {"product_name": "Ruffles", "category_id": 3, "category_name": "Snacks", "price": 4.49, "stock": 76, "x": 10.0, "y": 5.0, "z": 2,},
        {"product_name": "Cheetos", "category_id": 3, "category_name": "Snacks", "price": 3.79, "stock": 32, "x": 10.0, "y": 5.0, "z": 3,},
    ],
    "order_id": [
        # {
        #     "order_ID": "AAA000O1001",
        #     "member_ID": "AAA000",
        #     "items": [{"product_id": 1, "qty": 2}],
        #     "timestamp": "2026-03-05T12:00:00Z",
        #     "status": "RECEIVED",
        #     "result": None,
        # },
    ],
    "restock_id": [
        # {
        #     "restock_ID": "EMP001R1001",
        #     "employee_ID": "EMP001",
        #     "items": [{"product_id": 1, "qty": 10}],
        #     "timestamp": "2026-03-05T12:10:00Z",
        #     "status": "RECEIVED",
        #     "result": None,
        # },
    ],
}

TABLES = ["inventory", "customer", "employee", "order_id", "restock_id"]


def print_table(cur, table_name):
    cur.execute(f"SELECT * FROM {table_name}")
    rows = cur.fetchall()
    cols = [desc[0] for desc in cur.description]

    print(f"\n=== {table_name} ===")
    print("columns:", ", ".join(cols))
    print("row_count:", len(rows))

    if not rows:
        print("(empty)")
        return

    for i, row in enumerate(rows, start=1):
        print(f"[{i}] {row}")


def insert_customers(cur, rows):
    for r in rows:
        cur.execute(
            """
            INSERT INTO customer (member_ID, first_name, last_name, order_ID)
            VALUES (%s, %s, %s, %s)
            ON CONFLICT (member_ID) DO NOTHING
            """,
            (r["member_ID"], r.get("first_name"), r.get("last_name"), r.get("order_ID")),
        )


def insert_employees(cur, rows):
    for r in rows:
        cur.execute(
            """
            INSERT INTO employee (employee_ID, first_name, last_name, restock_ID)
            VALUES (%s, %s, %s, %s)
            ON CONFLICT (employee_ID) DO NOTHING
            """,
            (r["employee_ID"], r.get("first_name"), r.get("last_name"), r.get("restock_ID")),
        )


def insert_inventory(cur, rows):
    for r in rows:
        cur.execute(
            """
            INSERT INTO inventory (product_name, category_id, category_name, price, stock, x, y, z)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
            """,
            (
                r["product_name"],
                r["category_id"],
                r.get("category_name"),
                r["price"],
                r["stock"],
                r["x"],
                r["y"],
                r["z"],
            ),
        )


def insert_orders(cur, rows):
    for r in rows:
        cur.execute(
            """
            INSERT INTO order_id (order_ID, member_ID, items, timestamp, status, result)
            VALUES (%s, %s, %s, %s, %s, %s)
            ON CONFLICT (order_ID) DO NOTHING
            """,
            (
                r["order_ID"],
                r["member_ID"],
                Json(r["items"]),
                r.get("timestamp"),
                r.get("status", "RECEIVED"),
                r.get("result"),
            ),
        )


def insert_restocks(cur, rows):
    for r in rows:
        cur.execute(
            """
            INSERT INTO restock_id (restock_ID, employee_ID, items, timestamp, status, result)
            VALUES (%s, %s, %s, %s, %s, %s)
            ON CONFLICT (restock_ID) DO NOTHING
            """,
            (
                r["restock_ID"],
                r["employee_ID"],
                Json(r["items"]),
                r.get("timestamp"),
                r.get("status", "RECEIVED"),
                r.get("result"),
            ),
        )


def load_data(cur, data):
    insert_customers(cur, data.get("customer", []))
    insert_employees(cur, data.get("employee", []))
    insert_inventory(cur, data.get("inventory", []))
    insert_orders(cur, data.get("order_id", []))
    insert_restocks(cur, data.get("restock_id", []))


def main():
    conn = psycopg2.connect(
        dbname="grocery_inventory",
        user="grocerybot",
        password="team21",
        host="localhost",
        port="5432",
    )
    cur = conn.cursor()

    try:
        # load_data(cur, DATA_TO_INSERT)
        # conn.commit()
        # print("Data insert complete.")
        for table in TABLES:
            print_table(cur, table)
    except Exception:
        conn.rollback()
        raise
    finally:
        cur.close()
        conn.close()


if __name__ == "__main__":
    main()
