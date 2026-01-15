import firebase_admin
from firebase_admin import firestore

cred = firebase_admin.credentials.Certificate("credential.json")
firebase_admin.initialize_app(cred)

db = firestore.client()

def login(username, password):
    try:
        doc_ref = db.collection("account").document(username)
        doc = doc_ref.get()

        # Username not found
        if not doc.exists:
            return {
                "access": False,
                "error": "Username not found"
            }

        data = doc.to_dict()

        # Password mismatch
        if data["password"] != password:
            return {
                "access": False,
                "error": "Invalid password"
            }

        # Success
        return {
            "access": data["access_level"],
            "role": data.get("role", "unknown")
        }

    except Exception as e:
        # Database / connection error
        return {
            "access": False,
            "error": f"System error: {str(e)}"
        }
    
def create_new_account(username, password, first_name, last_name, access_level):
    db.collection("account").document(username).set({
        "username": username,
        "password": password,
        "first_name": first_name,
        "last_name": last_name,
        "access_level": access_level
    })

def add_product(product_id, name, price, stock, aisle, rack, shelf_level):
    db.collection("grocery_inventory").document(product_id).set({
        "name": name,
        "price": price,
        "stock": stock,
        "aisle": aisle,
        "rack": rack,
        "shelf_level": shelf_level
    })

def get_product(product_id):
    doc = db.collection("grocery_inventory").document(product_id).get()
    if doc.exists:
        return doc.to_dict()
    return None

def update_stock(product_id, qty):
    ref = db.collection("grocery_inventory").document(product_id)
    ref.update({
        "stock": firestore.Increment(-qty)
    })

# Create New Account
create_new_account("pree01", "1234", "p", "s", "employee")

# No Username
result = login("pree010101", "1234")
print(result)

# Wrong Password
result = login("pree01", "1235")
print(result)

# Success
result = login("pree01", "1234")
print(result)

product = get_product("apple_001")
print(product)

# decrement stock
add_product(
    product_id="002",
    name="Apple",
    price=20,
    stock=120,
    aisle="A2",
    shelf_level=1,
    rack=3
)

product = get_product("002")
print(product)

update_stock("002", 1)

product = get_product("002")
print(product)