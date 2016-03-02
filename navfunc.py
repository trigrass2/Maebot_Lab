"""
FILE: navfunc.py

  This files defines the NavFunc class, a class for efficiently
  computing navigation functions in a 2D plane. This plane is
  represented with complex numbers.
  
  The construction of the navigation functions follows the
  classical paper
    Robot Navigation Functions on Manifolds with Boundary
    DANIEL E. KODITSCHEK AND ELON RIMON
    ADVANCES IN APPLIED MATHEMATICS Vol. 11, pp. 412-442 (1990)
    
  Objects defined:
    class NavFunc
    function lectureFigures
    function randomNav
  
  For usage of NavFunc, see the source code of the two functions
  
  (c) 2015 Shai Revzen, released under GPL v. 3
"""

from pylab import *

def _asCArray( seq ):
  """(private)
  Convert parameter to a complex array.
  
  First converts to float array. If real, and last index of
  size 2, uses it as real and imaginary parts.
  """
  a = asarray(seq,dtype=complex)
  if a.shape[-1]==2 and not any(a.imag.flatten()):
     a = a[...,0] + 1j* a[...,1]
  return a
  
class NavFunc( object ):
  def __init__(self):
    self.ctr = asarray([0j])
    self.scl = asarray([1.0])
    self.goal = asarray([0+0j])
    self.kappa = 1
    self.lam = 1
  
  def addObs( self, xy, r ):
    """
    Add obstacles to the navigation function map
    INPUT:
      xy -- N or Nx2 -- centers of circular obstacles
      r -- N -- radii of circular obstacles
    """
    xy = _asCArray(xy)
    assert xy.ndim<2
    r = asfarray(r)
    assert r.size == xy.size
    assert all(r>0)
    self.ctr = concatenate( [self.ctr,xy],0 )
    self.scl = concatenate( [self.scl,r*r],0 )
  
  def setGoal( self, goal ):
    """Set navigation goal"""
    self.goal = _asCArray(goal)
  
  def _gamma( self, xy ):
    """
    Compute the gamma values at all (complex) points xy
    INPUT:
      xy -- N... -- complex array of any shape
      
    OUPUT: gamma
      Shaped the same as the input, with an additional index 
      added for goals if there are multiple goals
      
    See Rimon & Koditschek 1992, pp. 507, eqn. (6), but omitting
    the raising to the kappa power
    """
    xy = asarray(xy)
    sh = xy.shape
    xy = xy.reshape((xy.size,1))
    d = abs(xy - self.goal[newaxis,:])
    xy.shape = sh
    gamma = real(d*d)
    gamma.shape = sh + self.goal.shape
    return gamma
    
  def _beta( self, xy):
    """
    Compute the beta values at all (complex) points xy
    INPUT:
      xy -- N... -- complex array of any shape
      
    OUPUT: beta 
      Shaped the same as the input, with 1's appended to match self.goal
      
    See Rimon & Koditschek 1992, pp. 507, bottom left column
    """
    xy = asarray(xy)
    sh = xy.shape
    xy = xy.reshape((xy.size,1))
    d0 = abs(xy - self.ctr[newaxis,:])
    xy.shape = sh
    d1 = d0*d0 - self.scl[newaxis,:]
    d1[:,0] = -d1[:,0]
    beta = prod(d1,1).real
    beta.shape = sh + (1,) * self.goal.ndim
    return beta
  
  def nav(self, xy):
    """
    Compute the navigation function values at all (complex) 
    points xy
    
    INPUT:
      xy -- N... -- complex array of any shape
      
    OUPUT: navigation function 
      Shaped the same as the input x self.goal
      
    See Rimon & Koditschek 1992, pp. 508, eqn. (10)
    """
    g = self._gamma(xy)
    b = self._beta(xy)
    den = (g**self.kappa+self.lam*b)
    # Lets make sure than no negative denominators exist
    #  since we are potentially taking their kappa-th root
    nden = den<0
    if any(nden):
      g[nden] = -g[nden]
      den[nden] = -den[nden]
    phi = g / (den**(1.0/self.kappa))
    return phi
  
  def partial(self, xy, dxy, atol=1e-16, ltol=0.1, ftol=1e-16):
    """
    Compute the partial derivative of the navigation function at points xy
    in the direction (and initial scale) set by dxy
    
    INPUT:
      xy -- [...M x N...] -- locations to compute partial derivative at
      dxy -- [ N... ] -- direction(s) for partial derivative
      atol -- real > 0 -- absolute tolerance (minimal step size) used
      ltol -- 0 < real <0.5 -- tolerance for non-linearity
      ftol -- real > 0 -- function value differences to be treated as 0
    NOTE: dxy can be any array whose shape is compatible with that of xy
    according to the NumPy broadcasting rules. In particular, a scalar.
    """
    # We will need xy to have an extra index for broadcasting later on
    xy = asarray(xy)
    if xy.ndim:
        xy = xy[...,newaxis]
    else: # scalars become 1x1 arrays
        xy.shape = (1,1)
    #print ">>> ",xy.size,xy.ndim
    # broadcast dxy to the size of xy (without the extra index)
    dxy = (asarray(dxy) + zeros_like(xy[...,0]))[...,newaxis]
    z = zeros_like(dxy)
    # Compute nav function before and after each point in xy
    #   with offset dxy and -dxy
    xpn = concatenate([z,dxy,-dxy],-1)+xy
    f = self.nav(xpn.T)
    c,dp,dn = f
    # Compute changes in positive and negative dxy directions
    dp -= c; dp = dp.T
    dn -= c; dn = dn.T
    # Figure out which (if any) entries are too nonlinear
    #   for perfectly linear function, the ratio in nl will be 0
    #   We mark a point as nonlinear if it is nonlinear for any goal
    den = (abs(dp)+abs(dn))
    # Make sure that where den is effectively zero, we get nl==False
    den[den<ftol] = 1
    nl = any(abs(dp+dn)/den>ltol,0)
    # Ignore nonlinearity if step size is not greater than atol
    nl &= (abs(dxy[...,0])>atol)
    # Compute result
    res = (dp-dn)/(2*abs(dxy[...,0]))
    if any(nl):
      if sum(nl) == 1:
        # NumPy is inconsistent when only one element is returned
        res[:,nl] = self.partial(xy[nl],dxy[nl]/2.0,atol,ltol)[...,0]
      else:
        # recursively refine result in positions indicated by nl
        res[:,nl] = self.partial( 
          xy[nl].squeeze(),
          dxy[nl].squeeze()/2.0,
          atol,ltol
        )
    return res

  def heading(self,y,scl=1e-3):
    """
    Compute unit vector in the direction of motion at position y.
    INPUT:
      y -- position to compute heading at
      scl -- real>0 -- scale to use for gradient computations
    """
    y = asarray(y)
    yc = y[...,0]+1j*y[...,1]
    vx = self.partial(yc,scl)
    vy = self.partial(yc,scl*1j)
    vv = sqrt(vx*vx+vy*vy)
    res = asfarray((-vx/vv,-vy/vv))
    return res.squeeze()

def lectureFigures():
  """
  Prepare figures shown in lecture
  """
  t = linspace(-1.1,1.1,511)
  xx = linspace(-1.1,1.1,1<<16)
  x,y = meshgrid(t,t)
  xy = x + 1j* y
  nf = NavFunc()
  nf.goal = asarray([0.1])
  nf.addObs( [.4,-.4,-.4j,.4j,.8], 
             [0.15,0.15,0.15,0.15,0.075] )
  figure(1); clf()
  figure(2); clf()
  figure(3); clf()
  nav = []
  for k in xrange(2):
    for l in xrange(2):
      nf.kappa = 3+7*k
      nf.lam = 40*l+10
      nav.append(nf.nav(xy).squeeze())
      # Contour plot
      figure(2); subplot(2,2,k*2+l+1); cla()
      imshow( log(nav[-1]), extent=[t[0],t[-1],t[0],t[-1]])
      contour( xy.real, xy.imag[::-1,:], nav[-1], linspace(0,1,11) )
      hlines(0,t[0],t[-1])
      vlines(-.7,t[0],t[-1])
      grid(1); axis([t[0],t[-1],t[0],t[-1]]); axis('equal')
      title("$\kappa=%g, \lambda=%g$" % (nf.kappa,nf.lam))
      # Section along x axis
      figure(1)
      yy = nf.nav(xx)
      plot(xx,yy,',')
      # Section along y axis at x=-.7
      figure(3)
      yy = nf.nav(-0.7+xx*1j)
      plot(xx,yy,',')
  # Finish preparing figures
  figure(1)
  hlines([0,1],xx[0],xx[-1],'k')
  axis([xx[0],xx[-1],-.1,1.1])
  grid(1)
  figure(3)
  hlines([0,1],xx[0],xx[-1],'k')
  axis([xx[0],xx[-1],-.1,1.1])
  grid(1)

def randomNav(numTraj=50, tEnd = 10.0, dt=0.05):
  """
  Create a random navigation function, and
  generate numTraj random trajectories on that plot
  """
  # Create contour plot mesh
  t = linspace(-1.1,1.1,511)
  x,y = meshgrid(t,t)
  xy = x + 1j* y
  # Create NavFunc-tion
  nf = NavFunc()
  nf.goal = asarray([0])
  # Add 10 randomly placed obstacles
  for k in xrange(10):
    obs = (rand()*0.4+0.2)*exp(6.28j*rand())
    nf.addObs( [obs], [0.075])
  # Set "reasonable" params
  nf.kappa = int(k*1.5)
  nf.lam = k*10
  # Show contours
  figure(9); clf()
  f = nf.nav(xy).squeeze()
  contour(xy.real, xy.imag, f,linspace(0,1,21))
  # Show flow directions
  xyd = xy[::20,::20]
  xyd =xyd.reshape((xyd.size,))
  vx = nf.partial(xyd,0.01)[0]
  vy = nf.partial(xyd,0.01j)[0]
  vv = sqrt(vx*vx+vy*vy)
  vx = vx/vv
  vy = vy/vv
  plot(c_[xyd.real, xyd.real-vx/30].T
      ,c_[xyd.imag,xyd.imag-vy/30].T
      ,'-k',alpha=0.3)
  plot(xyd.real,xyd.imag,'.k')
  axis('equal')
  from time import sleep
  for k in xrange(numTraj):
    # look for valid initial point
    f0 = 1
    while f0<0 or f0>=1:
      y0 = (rand()*0.2+0.6)*exp(6.28j*rand())
      f0 = nf.nav(y0)[0]
    # Integrate trajectory from that starting point
    t = arange(0,tEnd,dt)
    yy = [(y0.real,y0.imag)]
    for ti in t:
      y = yy[-1]
      h = nf.heading(y)
      yy.append(y+h*dt)
    yy = asarray(yy)
    # Display
    plot(yy[:,0], yy[:,1], '.-')
    draw()
    sleep(0.1)
    
if __name__=="__main__":
  # Create lecture figures  
  lectureFigures()
  # Animate random navigation problems
  while True:
    randomNav()