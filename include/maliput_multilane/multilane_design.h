/// @file multilane_design.h
/// @page multilane_design Multilane Design
/// @author Matt Marjanović
/// @author Chien-Liang Fok
/// @author Agustin Alba Chicar
/// @date April 21, 2020
/// @tableofcontents
///
/// @section concrete_implementation_maliput_multilane Concrete Implementation: `maliput::multilane`
///
/// So-named because it admits multiple `Lanes` per
/// `Segment`, an advance over its predecessor (`monolane`) which only
/// admitted a single `Lane` per `Segment`.
///
/// `multilane`  is an implementation of the
/// `maliput` geometry API which synthesizes a road network from a small set
/// of primitive building blocks, mimicking techniques used in the geometric
/// design of real roads.  The basic geometry of a `Segment` is derived
/// from the combination of a plane curve, an elevation
/// function, and a superelevation function, combined together to define a
/// ruled surface.  A `Segment` has a longitudinal *reference curve*
/// (similar to a `Lane`'s centerline) and each of the `Lanes` of a
/// `Segment` is defined via a constant lateral offset, along the segment
/// surface, from that reference curve.
///
/// Three coordinate frames are involved in the following discussion:
///  * @f$(x,y,z)@f$ is a position in the `World`-frame.
///  * @f$(s,r,h)_{LANE,i}@f$ is a position in the `Lane`-frame (discussed
///    in section @ref world_frame_versus_lane_frame ) of the `Lane` with
///    index @f$i@f$.
///  * @f$(p,r,h)_{SEG}@f$ is a position in a curvilinear reference frame of
///    the `Segment`, analogous to @f$(s,r,h)_{LANE,i}@f$ for a `Lane`.
///    The parameter @f$p_{SEG} \in [0, 1]@f$ spans the `Segment` longitudinally.
///    @f$r_{SEG}@f$ is a lateral offset from the `Segment`'s reference curve,
///    along the `Segment` surface. @f$h_SEG@f$ is height above the surface.
///
/// @subsection segment_geometry `Segment` Geometry
///
/// > TODO Reconsider the use of the word "geometry" below.
/// > The geometry of a `Segment` is completely derived from a map
/// >
/// > @f[
/// > W: (p,r,h)_{SEG} \mapsto (x,y,z)
/// > @f]
/// >
/// > which we will construct in stages, starting with the `Segment` reference curve
/// >
/// > @f[
/// > W(p_{SEG}) \equiv W(p_{SEG},0,0),
/// > @f]
/// >
/// > followed by the `Segment` surface
/// >
/// > @f[
/// > W(p_{SEG},r_{SEG}) \equiv W(p_{SEG},r_{SEG},0).
/// > @f]
///
/// The construction of @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ will involve
/// three fundamental functions, @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$.
///
/// The first fundamental function @f$G_\text{xy}@f$ defines a two dimensional
/// *planar primitive curve* in the @f$xy@f$ -plane:
///
/// @f[
/// G_{xy}: p_{SEG} \mapsto (x,y).
/// @f]
///
/// This curve establishes the basic geometric primitive of the `Segment`
/// (e.g., "constant-radius arc").
/// We define @f$l@f$ as a path-length along this plane curve, in the range
/// @f$[0, l_\text{max}]@f$, where @f$l_\text{max}@f$ is the total path-length
/// of the curve.  @f$G_{xy}@f$ is specifically parameterized such that
///
/// @f[
/// p_{SEG} \equiv \frac{l}{l_\text{max}};
/// @f]
///
/// in other words, @f$p_{SEG}@f$ is linear in path-length along the planar
/// primitive curve and @f$p_{SEG} \in [0,1]@f$.
///
/// The second fundamental function @f$G_z@f$ specifies elevation above the
/// @f$(xy)@f$-plane (albeit with a peculiar scale factor):
///
/// @f[
/// G_z: p_{SEG} \mapsto \frac{1}{l_\text{max}}z
/// @f]
///
/// Taking @f$G_{xy} = (G_x, G_y)@f$ and @f$G_z@f$ together,
///
/// > @f[
/// > \left(\begin{array}{c} G_{xy}\\ l_\text{max}G_z \end{array}\right): p_{SEG} \mapsto
/// >   \left(\begin{array}{c}x\\y\\z\end{array}\right)
/// > @f]
///
/// @f[
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG}) =
/// \left(\begin{array}{c} G_x(p_{SEG})\\
///                        G_y(p_{SEG})\\
///                        l_\text{max}G_z(p_{SEG}) \end{array}\right)
/// @f]
///
/// defines the three dimensional *reference curve* @f$W(p_{SEG})@f$ for the `Segment`.
/// @f$G_z@f$ is constructed with the scale factor of @f$1/l_\text{max}@f$ specifically
/// so that:
///
/// @f[
/// \begin{eqnarray*}
///       z & = & l_\text{max} G_z(p_{SEG})\\
///         & = & l_\text{max} G_z\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{z} & = & \frac{dz}{dl} = \frac{d}{dp_{SEG}}G_z(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// This allows us to derive the first derivative of @f$G_z@f$ directly from
/// the `World`-frame slope @f$\dot{z} = \frac{dz}{dl}@f$ of the segment
/// surface along its reference curve.  This is convenient because @f$\dot{z}@f$
/// is what a road designer would nominally specify as the "slope of the road"
/// or the "grade of the road".
///
/// The third fundamental function @f$\Theta@f$ specifies the superelevation of
/// the `Segment` surface:
///
/// @f[
/// \Theta: p_{SEG} \mapsto \frac{1}{l_\text{max}}\theta
/// @f]
///
/// Superelevation @f$\theta@f$ is the "twist" in a road, given as a right-handed
/// angle of rotation around the tangent of the reference curve @f$W(p_{SEG})@f$.
/// Zero superelevation leaves the surface parallel with the
/// @f$xy@f$ plane. Note that superelevation becomes ambiguous when the
/// tangent of the reference curve points in the @f$\hat{z}@f$ direction.
///
/// As with @f$G_z@f$, @f$\Theta@f$ is scaled so that:
///
/// @f[
/// \begin{eqnarray*}
///       \theta & = & l_\text{max} \Theta\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{\theta} & = &
///               \frac{d\theta}{dl} = \frac{d}{dp_{SEG}}\Theta(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// > With the three fundamental functions in hand, we can express the orientation
/// > of the @f$(\hat{p},\hat{r},\hat{h})_{SEG}@f$ frame along the reference curve,
/// > with respect to the `World`-frame, as a roll/pitch/yaw rotation:
///
/// We use all three fundamental functions to define a rotation
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG}) &=
///  \mathbf{R}_{\gamma(p_{SEG})}
///  \mathbf{R}_{\beta(p_{SEG})} \mathbf{R}_{\alpha(p_{SEG})}
/// \end{align*}
/// @f]
///
/// where
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\gamma(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   \cos\gamma & -\sin\gamma & 0 \\
///   \sin\gamma &  \cos\gamma & 0 \\
///            0 &           0 & 1
///   \end{array}\right) & \text{(yaw)}\\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\beta(p_{SEG})}  &=
///   \left(\begin{array}{rrr}
///    \cos\beta & 0 & \sin\beta \\
///            0 & 1 &         0 \\
///   -\sin\beta & 0 & \cos\beta
///   \end{array}\right) & \text{(pitch)} \\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\alpha(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   1 &          0 &           0 \\
///   0 & \cos\alpha & -\sin\alpha \\
///   0 & \sin\alpha &  \cos\alpha
///   \end{array}\right) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// and
///
/// @f[
/// \begin{align*}
/// \gamma(p_{SEG}) &=
///   \mathrm{atan2}\negthickspace\left(\frac{dG_y}{dp_{SEG}},
///                       \frac{dG_x}{dp_{SEG}}\right) & \text{(yaw)}\\
/// \beta(p_{SEG})  &=
///   \arctan\negthickspace\left(\frac{dG_z}
///                                         {dp_{SEG}}\right)
/// & \text{(pitch)} \\
/// \alpha(p_{SEG}) &= l_\text{max}\Theta(p_{SEG}) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// > Note that @f$\hat{p}_{SEG}@f$ is solely determined by @f$W(p_{SEG})@f$,
/// > and as expected,
/// > @f$\hat{p}_{SEG} = \frac{W'(p_{SEG})}{\lVert W'(p_{SEG})\rVert}@f$.
///
/// With @f$\mathbf{R}(p_{SEG})@f$ , we can extend the `Segment` reference curve @f$W(p_{SEG})@f$
/// to construct the `Segment` *surface* @f$W(p_{SEG}, r_{SEG})@f$ as:
///
/// @f[
/// \begin{align*}
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG},r_{SEG}) = \left(
/// \begin{array}{c}
///    G_{xy}(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ 0 \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This function defines a *ruled surface*.  For any @f$p_{SEG}@f$,
/// @f$W(p_{SEG},r_{SEG})@f$ is linear in @f$r_{SEG}@f$ and motion along
/// @f$r_{SEG}@f$ is in a straight line.
///
/// Now that we have the surface embedding @f$W(p_{SEG},r_{SEG})@f$,
/// we can derive
/// the basis vectors @f$(\hat{p}, \hat{r}, \hat{h})_{SEG}@f$ along the surface
/// and the corresponding orientation @f$\mathbf{R}(p_{SEG},r_{SEG})@f$:
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG},r_{SEG}) &=
///                      \begin{pmatrix}\hat{p} & \hat{r} & \hat{h}\end{pmatrix}\\
/// \hat{p}_{SEG} &=
///  \frac{\partial_{p_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{p_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{r}_{SEG} &=
///  \frac{\partial_{r_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{r_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{h}_{SEG} &= \hat{p}_{SEG} \times \hat{r}_{SEG}
/// \end{align*}
/// @f]
///
/// A few things are worth noting at this point:
///
///  * @f$\hat{r}_{SEG} = \mathbf{R}(p_{SEG}) \begin{pmatrix}0\\1\\0\end{pmatrix}@f$.
///    Thus, @f$\hat{r}_{SEG}@f$ is independent of @f$r_{SEG}@f$.
///  * @f$\mathbf{R}(p_{SEG},r_{SEG}) = \mathbf{R}(p_{SEG})@f$ along
///    @f$r_{SEG} = 0@f$ just as it should be; the orientation along the
///    `Segment`'s reference curve is consistent in both expressions.
///  * @f$\hat{p}_{SEG}@f$ is *not necessarily* independent of
///    @f$r_{SEG}@f$.  Consequently, @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ is not
///    necessarily equal to @f$\mathbf{R}(p_{SEG})@f$ for
///    @f$r_{SEG}\ne 0@f$.  This will become important when we try to
///    join `Segments` end-to-end preserving @f$G^1@f$ continuity, discussed in
///    section @ref ensuring_g1_contiguity .
///
/// *Finally*, with @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ in hand (and points 1 and
/// 2 above), we can define the complete volumetric world map
/// @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ for a `Segment`'s geometry:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix}x\\y\\z\end{pmatrix} = W(p_{SEG},r_{SEG},h_{SEG}) = \left(
/// \begin{array}{c}
///    G_x(p_{SEG})\\
///    G_y(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG},r_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ h_{SEG} \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This is simply @f$W(p_{SEG},r_{SEG})@f$ displaced by @f$h_{SEG}@f$ along
/// the surface normal @f$\hat{h}_{SEG}@f$.
///
/// @subsection lane_geometry `Lane` Geometry
///
/// A `Lane` derives its geometry from its `Segment`.  In `multilane`, the
/// centerline of the `Lane` with index @f$i@f$ is a parallel curve with a constant
/// lateral offset @f$r_i@f$ from the reference curve (at @f$r_{SEG} = 0@f$) of the
/// `Segment`.  We can express this relationship as a transform between
/// @f$(s,r,h)_{LANE,i}@f$ (`Lane`-frame) and @f$(p,r,h)_{SEG}@f$
/// (`Segment`-frame):
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} p_{SEG}\\
///                 r_{SEG}\\
///                 h_{SEG} \end{pmatrix}
/// &= \begin{pmatrix}    P(s_{LANE,i})\\
///                    r_{LANE,i} + r_i\\
///                          h_{LANE,i} \end{pmatrix}
/// \end{align*}
/// @f]
///
/// The tricky part here is @f$P:s_{LANE,i} \mapsto p_{SEG}@f$, which relates
/// @f$s_{LANE,i}@f$ to @f$p_{SEG}@f$, and involves the
/// path-length integral over @f$W(p_{SEG},r_{SEG})@f$.
///
/// `maliput` defines @f$s_{LANE,i}@f$ as the path-length along a `Lane`'s
/// centerline, and in `multilane` that centerline is a curve with constant
/// @f$r_{SEG} = r_i@f$.  Thus:
///
/// @f[
/// \begin{align*}
/// s_{LANE,i} = S(p_{SEG}) &=
///  \left. \int \left\lVert \partial_{p_{SEG}}W(p_{SEG}, r_{SEG})
///  \right\rVert dp_{SEG} \right\rvert_{r_{SEG} = r_i}.
/// \end{align*}
/// @f]
///
/// The function @f$P@f$ that we need is the inverse of the path-integral @f$S@f$.
///
/// Unfortunately, there is generally no closed-form solution for either
/// @f$S@f$ or @f$P@f$, particularly if the surface is not flat.  `multilane` will
/// compute @f$P(s_{LANE,i})@f$ and @f$S(p_{SEG})@f$ analytically if
/// possible (e.g., for some flat surfaces) and otherwise will use more costly
/// numerical methods to ensure accurate results. Which makes us
/// wonder, perhaps the `Lane`-frame of `maliput` would be better off
/// using an arbitrary longitudinal parameter @f$p_{LANE,i}@f$ which could
/// be converted to a distance @f$s_{LANE,i}@f$ on demand, instead of the other
/// way around.
///
/// > TODO: Derivation of orientation at arbitrary @f$(s,r,h)_{LANE,i}@f$ point.
/// >
/// > TODO: Derivation of motion-derivatives.
/// >
/// > TODO: Derivation of surface/path curvatures.
///
/// ### Available Implementations of @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$
///
/// `multilane` currently implements one form for each of @f$G_{xy}@f$,
/// @f$G_z@f$, and @f$\Theta@f$.  @f$G_{xy}@f$ is implemented for a constant curvature
/// arc (which includes zero curvature, i.e., straight line segments).
/// Elevation @f$G_z@f$ and superelevation @f$\Theta@f$ are implemented for cubic
/// polynomials.  These forms were chosen because they provide the smallest,
/// simplest set of primitives that allow for the assembly of fully
/// three-dimensional road networks that maintain @f$G^1@f$ continuity across
/// segment boundaries.
///
/// The exact form that @f$G_{xy}@f$ takes is:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} x\\ y \end{pmatrix} = G_\text{xy}(p_{SEG}) &=
///     \begin{pmatrix}x_0\\ y_0\end{pmatrix} +
///     \left\lbrace \begin{array}
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \cos(\gamma_0 - \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \sin(\gamma_0 - \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa > 0\\
///         l_\text{max} p_{SEG}
///           \begin{pmatrix}\cos{\gamma_0}\\ \sin{\gamma_0}\end{pmatrix}
///           & \text{for }\kappa = 0\\
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \cos(\gamma_0 + \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \sin(\gamma_0 + \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa < 0\\
///     \end{array} \right\rbrace
/// \end{align*}
/// @f]
///
///
/// where @f$\kappa@f$ is the signed curvature (positive is
/// counterclockwise/leftward), @f$l_\text{max}@f$ is the arc length,
/// @f$\begin{pmatrix}x_0\\y_0\end{pmatrix}@f$ is the
/// starting point of the arc, and @f$\gamma_0@f$ is the initial yaw of the
/// (tangent) of the arc (with @f$\gamma_0 = 0@f$ in the @f$+\hat{x}@f$
/// direction).  Note that the @f$\kappa = 0@f$ expression is simply a line
/// segment of length @f$l_\text{max}@f$, and it is the limit of the @f$\kappa
/// \neq 0@f$ expressions as @f$\kappa \to 0@f$.
///
/// With regards to geometric road design, a constant curvature
/// @f$G_\text{xy}@f$ does not provide a complete toolkit.  Most road designs
/// involve clothoid spirals, which are plane curves with curvature that
/// is /linear/ in path length.This is so that vehicles can navigate
/// roads using continuous changes in steering angle, and, likewise, so that
/// their occupants will experience continuous changes in radial acceleration.
/// `multilane` is expected to extend support for clothoid @f$G_\text{xy}@f$
/// in the future.
///
/// For @f$G_z@f$ and @f$\Theta@f$, a cubic polynomial is the lowest-degree polynomial
/// which allows for independently specifying the value and the first derivative
/// at both endpoints.  Thus, @f$G_z@f$ takes the form:
///
/// @f[
/// \begin{align*}
/// \begin{split}
/// \frac{1}{l_\text{max}}z = G_z(p_{SEG}) &=
///  \frac{z_0}{l_\text{max}} +
///  \dot{z_0} p_{SEG} +
///  \left(\frac{3(z_1 - z_0)}{l_\text{max}} - 2\dot{z_0} - \dot{z_1}\right)
///    p_{SEG}^2 \\
///  &\quad + \left(\dot{z_0} + \dot{z_1} - \frac{2(z_1 - z_0)}{l_\text{max}}\right)
///    p_{SEG}^3
/// \end{split}
/// \end{align*}
/// @f]
///
/// where @f$z_0@f$ and @f$z_1@f$ are the initial and final elevation
/// respectively, and @f$\dot{z_0}@f$ and @f$\dot{z_1}@f$ are the initial and
/// final @f$\frac{dz}{dl}@f$, which is simply the slope of the road as
/// measured by the intuitive "rise over run".  @f$\Theta@f$ has an identical
/// expression, with every @f$z@f$ replaced by @f$\theta@f$.  Note that
/// @f$\dot{\theta} = \frac{d\theta}{dl}@f$, the rate of twisting of the road,
/// is not particularly intuitive, but that's ok because in general
/// @f$\dot{\theta_0}@f$ and @f$\dot{\theta_1}@f$ will be set by `multilane` and
/// not by the road designer, as we will see in section @ref ensuring_g1_contiguity .
///
/// @subsection ensuring_g1_contiguity Ensuring G¹ Continuity
///
/// > TODO:  Tell me more!
///
/// @subsection builder_helper_interface `Builder` helper interface
///
/// Users are not expected to assemble a `multilane::RoadGeometry` by
/// constructing individual instances of `multilane::Lane`, etc, by hand.
/// Instead, `multilane` provides a `Builder` interface which handles
/// many of the constraints involved in constructing a valid `RoadGeometry`.
///
/// > TODO:  Tell me more!
///
/// @subsection yaml_file_format YAML file format
///
/// > TODO:  Tell me more!
