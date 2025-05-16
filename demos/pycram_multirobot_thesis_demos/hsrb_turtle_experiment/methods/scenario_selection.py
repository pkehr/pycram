class ScenarioSelection:
    def __init__(self):
        self.navigate_start_turtle = False
        self.navigate_start_hsrb = False
        self.navigate_table_one_hsrb = False

        self.transport_milk = False
        self.transport_coffee = False
        self.transport_chips = False

        self.navigate_table_two_turtle = False
        self.navigate_table_two_hsrb = False

    def set_demo_scenario(self,
                          use_hsr=True, use_turtle=False,
                          transport_milk=True, transport_coffee=True, transport_chips=True):
        if use_hsr:
            self.navigate_start_hsrb = True
            self.navigate_table_one_hsrb = True
            self.navigate_table_two_hsrb = True

        if use_turtle:
            self.navigate_start_turtle = True
            self.navigate_table_two_turtle = True

        if transport_milk:
            self.transport_milk = True

        if transport_coffee:
            self.transport_coffee = True

        if transport_chips:
            self.transport_chips = True

