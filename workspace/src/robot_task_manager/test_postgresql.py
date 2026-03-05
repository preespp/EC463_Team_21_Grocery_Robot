import psycopg2

conn = psycopg2.connect(
    dbname="grocery_inventory",
    user="grocerybotr",
    password="team21",
    host="localhost",
    port="5432"
)

cur = conn.cursor()

def add_item(name, category, quantity, location):
    cur.execute("""
        INSERT INTO inventory (name, category, quantity, location)
        VALUES (%s, %s, %s, %s)
    """, (name, category, quantity, location))
    conn.commit()

def delete_item(item_id):
    cur.execute("DELETE FROM inventory WHERE id = %s", (item_id,))
    conn.commit()

def update_quantity(item_id, new_quantity):
    cur.execute("""
        UPDATE inventory
        SET quantity = %s, last_updated = CURRENT_TIMESTAMP
        WHERE id = %s
    """, (new_quantity, item_id))
    conn.commit()

def get_item(name):
    cur.execute("SELECT * FROM inventory WHERE name = %s", (name,))
    return cur.fetchone()
