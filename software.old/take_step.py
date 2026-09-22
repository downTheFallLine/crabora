import crabora_bus as cb

mb = cb.MultiBus()
mb.close()
mb.open()

CENTER = cb.deg_to_pos(0)
UP = cb.deg_to_pos(-60)
DOWN = cb.deg_to_pos(60)

# raise leg 4 tibia and femur
mb.write_goal_move(43,DOWN,40)
mb.write_goal_move(42,DOWN,40)

# swing coxa
mb.write_goal_move(41,DOWN,40)

# lower tibia and femur
mb.write_goal_move(43,UP,40)
mb.write_goal_move(42,UP,40)


