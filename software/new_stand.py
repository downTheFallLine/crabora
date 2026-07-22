import crabora_bus as cb

mb = cb.multiBus()
mb.close()
mb.open()

CENTER = cb.deg_to_pos(0)
UP = cb.deg_to_pos(-60)
DOWN = cb.deg_to_pos(60)

# all femurs up!
mb.write_goal_move(12,UP,10)
mb.write_goal_move(22,UP,10)
mb.write_goal_move(32,UP,10)
mb.write_goal_move(42,UP,10)
mb.write_goal_move(52,UP,10)
mb.write_goal_move(62,UP,10)