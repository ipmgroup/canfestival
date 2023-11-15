from os import sep
s = ""
d = sep.join(__file__.split(sep)[:-1]) + sep
_ = lambda f: s.rstrip(open(d+f).read())
l = lambda f: s.split(_(f), "\n")

try:
    __doc__ = _('README')
except:
    pass
