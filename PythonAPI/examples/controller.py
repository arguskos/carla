try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_g
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')



# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    def __init__(self):
        self.forward = False
        self.right = False
        self.left = False  
        self.back = False
        self.action_dict = []
        self.enabled_keys = []
        # self.client 
        pass

    def add_action(self, key, actioin):
        assert key in self.action_dict, "key is already exist"
        self.action_dict[key] = action
    
    def remove_action(self, action):
        self.action_dict = {key:val for key, val in myDict.items() if val != action}

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if event.key in self.action_dict:
                    self.action_dict[event.key]()
        keys = pygame.key.get_pressed()
        self.forward = keys[K_UP] or keys[K_w]
        self.back = keys[K_DOWN] or keys[K_s]
        self.left = keys[K_LEFT] or keys[K_a]
        self.right = keys[K_RIGHT] or keys[K_d]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

