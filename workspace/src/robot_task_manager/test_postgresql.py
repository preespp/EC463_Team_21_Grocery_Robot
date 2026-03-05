import psycopg2

conn = psycopg2.connect(
    dbname="grocery_inventory",
    user="grocerybot",
    password="team21",
    host="localhost",
    port="5432"
)

cur = conn.cursor()


def customer_member_create(member_id, first_name, last_name):
    cur.execute("""
        INSERT INTO customer (member_id, first_name, last_name)
        VALUES (%s, %s, %s)
    """, (member_id, first_name, last_name))
    conn.commit()


def delete_customer(member_id, first_name, last_name):
    cur.execute("""
        DELETE FROM customer
        WHERE member_id = %s AND first_name = %s AND last_name = %s
    """, (member_id, first_name, last_name))
    conn.commit()


def employee_create(employee_id, first_name, last_name):
    cur.execute("""
        INSERT INTO employee (employee_id, first_name, last_name)
        VALUES (%s, %s, %s)
    """, (employee_id, first_name, last_name))
    conn.commit()


def delete_employee(employee_id, first_name, last_name):
    cur.execute("""
        DELETE FROM employee
        WHERE employee_id = %s AND first_name = %s AND last_name = %s
    """, (employee_id, first_name, last_name))
    conn.commit()


def add_new_product(product_name, category_id, category_name, price, stock, x, y, z):
    cur.execute("""
        INSERT INTO inventory (product_name, category_id, category_name, price, stock, x, y, z)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        RETURNING product_id
    """, (product_name, category_id, category_name, price, stock, x, y, z))
    product_id = cur.fetchone()[0]
    conn.commit()
    return product_id


def delete_product(product_id, product_name):
    cur.execute("""
        DELETE FROM inventory
        WHERE product_id = %s AND product_name = %s
    """, (product_id, product_name))
    conn.commit()


def update_quantity(product_id, new_quantity):
    cur.execute("""
        UPDATE inventory
        SET stock = %s
        WHERE product_id = %s
    """, (new_quantity, product_id))
    conn.commit()


def update_name(product_id, new_name):
    cur.execute("""
        UPDATE inventory
        SET product_name = %s
        WHERE product_id = %s
    """, (new_name, product_id))
    conn.commit()


def update_location(product_id, x, y, z):
    cur.execute("""
        UPDATE inventory
        SET x = %s, y = %s, z = %s
        WHERE product_id = %s
    """, (x, y, z, product_id))
    conn.commit()


def get_item(product_name):
    cur.execute("SELECT * FROM inventory WHERE product_name = %s", (product_name,))
    return cur.fetchone()


def run_samples():
    member_id = "AAA100"
    employee_id = "100AAA"
    first_name = "Pree"
    last_name = "Simphliphan"
    product_name = "Apple"

    print("1) customer_member_create")
    customer_member_create(member_id, first_name, last_name)
    print("   inserted customer:", member_id)

    print("2) employee_create")
    employee_create(employee_id, first_name, last_name)
    print("   inserted employee:", employee_id)

    print("3) add_new_product")
    product_id = add_new_product(product_name, 1, "Fruit", 0.99, 1, 1.0, 2.0, 2)
    print("   inserted product_id:", product_id)

    print("4) update_quantity")
    update_quantity(product_id, 25)
    print("   updated stock to 25")

    print("5) update_name")
    renamed = "sample_apple_renamed"
    update_name(product_id, renamed)
    print("   renamed product to:", renamed)

    print("6) update_location")
    update_location(product_id, 5.5, 6.6, 1)
    print("   updated location to x=5.5, y=6.6, z=1")

    print("7) get_item")
    row = get_item(renamed)
    print("   fetched row:", row)

    print("8) delete_product")
    delete_product(product_id, renamed)
    print("   deleted product:", product_id)

    print("9) delete_employee")
    delete_employee(employee_id, first_name, last_name)
    print("   deleted employee:", employee_id)

    print("10) delete_customer")
    delete_customer(member_id, first_name, last_name)
    print("   deleted customer:", member_id)


if __name__ == "__main__":
    try:
        run_samples()
    finally:
        cur.close()
        conn.close()
