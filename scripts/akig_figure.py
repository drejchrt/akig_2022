import random

from matplotlib import pyplot as plt



class AkigFigure():
    def __init__(self, title=None, label_x=None, label_y=None):
        self.fig, self.ax = plt.subplots()
        self.artists = []
        # artist = {
        #     'name':String
        #     'aritst': matplolib_aritist_object
        #     
        #}
        
        self.fig.suptitle(title if not title is None else 'Title')
        self.ax.set_xlabel(label_x if not label_x is None else 'x')
        self.ax.set_ylabel(label_y if not label_y is None else 'y')

    
    
    def add_artist(self, data, name=None, type='line', *args, **kwargs):
        if name == None:
            name = f'artist-{len(self.artists)}'

        if type=='line':
            line = self.ax.plot(data['x'],data['y'],*args, **kwargs)
            self.artists.append({
                'name': name,
                'artist': line
                })
            
         
        
    def update_artist(self,artist,values):
        pass

    def show(self):
        plt.show(block=False)

def test():
    x = list(range(10))
    y = [random.randint(0,10) for _ in range(10)]

    fig = AkigFigure(title='AkigFigure',label_x='[m]',label_y='[m]')
    fig.add_artist({'x':x,'y':y},name='Line111')
    
    fig.show()
    input('continue?')
    fig.add_artist({'x':x,'y':[random.randint(0,10) for _ in range(10)]},name='lein_55')


if __name__ == '__main__':
    test()
    input('Press any key to exit the program...')
