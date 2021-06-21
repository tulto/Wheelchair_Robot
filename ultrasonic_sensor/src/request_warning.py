

class WarningRequest: #defining a class in order to check if the Service received a warning request

    requested_warning = [False, False, False, False]
    got_warning_request = [False, False, False, False]

    @classmethod
    def requested_warning_msg(cls):
        return cls.requested_warning

    @classmethod
    def received_warning_request(cls):
        return cls.got_warning_request

    @classmethod
    def set_requested_warning(cls, list_of_true_echo_values):
        for bool_val in range(len(cls.requested_warning)):
            cls.requested_warning[bool_val]=(list_of_true_echo_values[bool_val] or cls.requested_warning[bool_val])

    @classmethod
    def setback_requested_warning(cls, list_of_true_echo_values):
        for bool_val in range(len(cls.requested_warning)):
            cls.requested_warning[bool_val]=(list_of_true_echo_values[bool_val] and cls.requested_warning[bool_val])

    @classmethod
    def set_received_warning_request(cls, index_num, received_a_request):
        cls.got_warning_request[index_num] = received_a_request

    @classmethod
    def setback_all_values(cls):
        cls.requested_warning = [False, False, False, False]
        cls.got_warning_request = [False, False, False, False]
