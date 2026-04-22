import numpy as np

def knife_edge_to_profile(x_pos, P):
    """Convert cumulative power P(x) into normalized intensity profile via gradient."""
    x = np.asarray(x_pos, float)
    P = np.asarray(P, float)
    dP = np.gradient(P, x)
    I = np.clip(dP, 0, None)
    I /= (I.max() if I.max() > 0 else 1.0)
    return x, I

def fit_gaussian_1d(x, y):
    """
    Fit y ≈ A*exp(-2*(x-x0)^2/w^2) + C
    Returns (x0, w, A, C, R2) or None.
    """
    x = np.asarray(x, float); y = np.asarray(y, float)
    y = np.clip(y, 0, None)
    if y.max() <= 0 or x.size < 5:
        return None
    A0 = float(y.max() - np.median(y))
    C0 = float(np.median(y))
    x0 = float(x[np.argmax(y)])
    # width guess
    hm = C0 + 0.5*A0
    above = np.where(y >= hm)[0]
    w0 = (x[above[-1]] - x[above[0]])/np.sqrt(2*np.log(2)) if above.size > 1 else (x.max()-x.min())/6
    w0 = max(abs(w0), 1e-6)

    params = np.array([A0, x0, w0, C0], float)
    for _ in range(40):
        A, x0, w, C = params
        w = max(w, 1e-6)
        E = np.exp(-2.0*((x-x0)**2)/(w*w))
        model = A*E + C
        r = y - model
        dA = E
        dx0 = A*E*(4*(x-x0)/(w*w))
        dw  = A*E*(4*((x-x0)**2)/(w**3))
        dC  = np.ones_like(x)
        J = np.vstack([dA, dx0, dw, dC]).T
        dp, *_ = np.linalg.lstsq(J, r, rcond=None)
        params += dp
        if np.linalg.norm(dp) < 1e-9:
            break

    A, x0, w, C = params
    model = A*np.exp(-2.0*((x-x0)**2)/(w*w)) + C
    ss_res = float(np.sum((y - model)**2))
    ss_tot = float(np.sum((y - y.mean())**2)) if y.size > 1 else 1.0
    R2 = 1.0 - ss_res/ss_tot if ss_tot > 0 else 1.0
    return float(x0), float(abs(w)), float(A), float(C), float(R2)

def d4sigma_radius(x, I):
    """ISO 11146 second-moment radius (1D). Returns (centroid, radius)."""
    x = np.asarray(x, float); I = np.asarray(I, float)
    I = np.clip(I, 0, None)
    S0 = np.trapezoid(I, x)
    if S0 <= 0: return np.nan, np.nan
    xc = np.trapezoid(x*I, x)/S0
    var = np.trapezoid(((x-xc)**2)*I, x)/S0
    r = 2*np.sqrt(var)  # radius = 2*sqrt(variance); diameter would be 4*sqrt(var)
    return float(xc), float(r)

def gaussian_2d_preview(x0, wx, y0, wy, amp=1.0, nx=240, ny=240, span=3.0):
    """Elliptical Gaussian preview grid centered at (x0,y0)."""
    x = np.linspace(x0 - span*wx, x0 + span*wx, nx)
    y = np.linspace(y0 - span*wy, y0 + span*wy, ny)
    X, Y = np.meshgrid(x, y)
    Z = amp*np.exp(-2.0*(((X-x0)**2)/(wx*wx) + ((Y-y0)**2)/(wy*wy)))
    return x, y, Z
