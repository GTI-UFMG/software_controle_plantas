import numpy as np

# Here is a simple test to check if the data is passed by reference or by value.
# The data is passed by reference as long as we do not create a new object.
# If we create a new object, then the data is passed by value.  

class TestClass:
    
    def __init__(self):
        self.data = np.array([1,2,2,2,2,5,6,7,8,9])

    def add_data(self, new_data):
        self.data = np.append(self.data, new_data)

    def get_data(self,indices=None):
        if indices is None:
            data2 = np.array(self.data.size)
            data2 = self.data[0:self.data.size]
            return data2
        else:
            return self.data[indices]  
    
    def print_data(self):
        print(self.data)
    
if __name__ == "__main__":
    test_obj = TestClass()
    
    v = test_obj.get_data(indices=[0,1,4])
    v[0:2] = -1.0
    
    print(v)   
    test_obj.print_data()
    
    print("\nNew test\n")
    
    v = test_obj.get_data()
    v[0:2] = -5.0
    print(v)
    test_obj.print_data()
    