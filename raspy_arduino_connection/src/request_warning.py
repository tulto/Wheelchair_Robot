

class WarningRequest: #defining a class in order to check if the Service received a warning request

    requested_warning = [True, True, True, True]
    got_warning_request = False

    @classmethod
    def requested_warning_msg(cls):
        return cls.requested_warning

    @classmethod
    def received_warning_request(cls):
        return cls.got_warning_request

    @classmethod
    def set_requested_warning(cls, list_of_true_echo_values):
        cls.requested_warning=list_of_true_echo_values

    @classmethod
    def set_received_warning_request(cls, received_a_request):
        cls.got_warning_request = received_a_request

    @classmethod
    def setback_all_values(cls):
        cls.requested_warning = [True, True, True, True]
        cls.got_warning_request = False
