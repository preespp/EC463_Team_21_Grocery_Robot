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

INSERT INTO inventory (product_name, category_id, category_name, price, stock, x, y, z) VALUES
  ('Milk', 1, 'Drinks', 3.99, 20, 1.762289, 0.161800, 1),
  ('Water', 1, 'Drinks', 1.49, 100, 1.762289, 0.161800, 2),
  ('Apple', 2, 'Fruits', 0.99, 10, 4.282289, 3.581800, 2),
  ('Orange', 2, 'Fruits', 1.19, 30, 4.282289, 3.581800, 1),
  ('Rays', 3, 'Snacks', 4.29, 201, 4.642289, -3.798200, 1),
  ('Ruffles', 3, 'Snacks', 4.49, 76, 4.642289, -3.798200, 2),
  ('Cheetos', 3, 'Snacks', 3.79, 32, 4.642289, -3.798200, 3)
ON CONFLICT DO NOTHING;
