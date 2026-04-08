INSERT INTO customer (member_id, first_name, last_name, order_id) VALUES
  ('AAA000', 'Pree', 'Simphliphan', NULL),
  ('AAA001', 'Lion', 'Jiang', NULL),
  ('AAA002', 'Feng', 'Tai', NULL),
  ('AAA003', 'Andy', 'Nguyen', NULL),
  ('AAA004', 'Darren', 'Sajino', NULL),
  ('AAA005', 'Bernie', 'Su', NULL)
ON CONFLICT (member_id) DO NOTHING;

INSERT INTO employee (employee_id, first_name, last_name, password, restock_id) VALUES
  ('000AAA', 'Pree', 'Simphliphan', 'team21', NULL),
  ('001AAA', 'Lion', 'Jiang', 'team21', NULL),
  ('002AAA', 'Feng', 'Tai', 'team21', NULL),
  ('003AAA', 'Andy', 'Nguyen', 'team21', NULL),
  ('004AAA', 'Darren', 'Sajino', 'team21', NULL),
  ('005AAA', 'Bernie', 'Su', 'team21', NULL)
ON CONFLICT (employee_id) DO NOTHING;

INSERT INTO inventory (product_id, product_name, category_id, category_name, price, stock, x, y, z) VALUES
  (1, 'Green Tea', 1, 'Drinks', 3.49, 20, 5.177849, -1.909191, 1),
  (2, 'Water', 1, 'Drinks', 1.49, 100, 5.186558, -1.682527, 2),
  (3, 'Apple', 2, 'Fruits', 0.99, 10, 2.760906, 0.894713, 2),
  (4, 'Orange', 2, 'Fruits', 1.19, 30, 2.550116, 0.885422, 1),
  (5, 'Lemon', 2, 'Fruits', 0.89, 18, 5.625196, 0.593373, 1),
  (6, 'Can', 1, 'Drinks', 2.29, 24, 5.467639, 0.614740, 2),
  (7, 'Bag of Chips', 3, 'Snacks', 4.29, 76, 5.793580, 0.572953, 3)
ON CONFLICT (product_id) DO UPDATE
SET
  product_name = EXCLUDED.product_name,
  category_id = EXCLUDED.category_id,
  category_name = EXCLUDED.category_name,
  price = EXCLUDED.price,
  stock = EXCLUDED.stock,
  x = EXCLUDED.x,
  y = EXCLUDED.y,
  z = EXCLUDED.z;

SELECT setval(
  pg_get_serial_sequence('inventory', 'product_id'),
  COALESCE((SELECT MAX(product_id) FROM inventory), 1),
  true
);
