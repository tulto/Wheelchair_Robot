class WarningRequest: #defining a class in order to check if the Service received a warning request

    def __init__(self):
        self.requested_warning = [True, True, True, True]
        self.got_warning_request = False
    
    @staticmethod
    def requested_warning_msg():
        return self.requested_warning
    
    @staticmethod
    def received_warning_request():
        return self.got_warning_request

    @staticmethod
    def set_requested_warning(list_of_true_echo_values):
        self.requested_warning=list_of_true_echo_values
    
    @staticmethod
    def set_received_warning_request(received_a_request):
        self.got_warning_request = received_a_request

    @staticmethod
    def setback_all_values():
        self.requested_warning = [True, True, True, True]
        self.got_warning_request = False
    
