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
  (1, 'GreenTea', 1, 'Drinks', 3.49, 20, 0.75, -0.27, 1),
  (2, 'Water', 1, 'Drinks', 1.49, 100, 0.75, 0.09, 3),
  (3, 'Chips', 3, 'Snacks', 4.29, 76, 0.75, -0.09, 2),
  (4, 'Can', 1, 'Drinks', 2.29, 24, 0.75, 0.27, 4)
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

DELETE FROM inventory WHERE product_id NOT IN (1, 2, 3, 4);

SELECT setval(
  pg_get_serial_sequence('inventory', 'product_id'),
  COALESCE((SELECT MAX(product_id) FROM inventory), 1),
  true
);
