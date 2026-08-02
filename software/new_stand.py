import crabora_bus as cb

mb = cb.MultiBus()
mb.close()
mb.open()

CENTER = cb.deg_to_pos(0)
UP = cb.deg_to_pos(-60)
DOWN = cb.deg_to_pos(60)


# all femurs up!
mb.write_goal_move(12,UP,40)
mb.write_goal_move(22,UP,40)
mb.write_goal_move(32,UP,40)
mb.write_goal_move(42,UP,40)
mb.write_goal_move(52,UP,40)
mb.write_goal_move(62,UP,40)

mb.write_goal_move(13,DOWN,40)
mb.write_goal_move(23,DOWN,40)
mb.write_goal_move(33,DOWN,40)
mb.write_goal_move(43,DOWN,40)
mb.write_goal_move(53,DOWN,40)
mb.write_goal_move(63,DOWN,40)

mb.write_goal_move(12,DOWN,40)
mb.write_goal_move(22,DOWN,40)
mb.write_goal_move(32,DOWN,40)
mb.write_goal_move(42,DOWN,40)
mb.write_goal_move(52,DOWN,40)
mb.write_goal_move(62,DOWN,40)

