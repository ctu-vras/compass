<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# magnetic\_model

This package contains utilities for working with the
[World Magnetic Model](https://en.wikipedia.org/wiki/World_Magnetic_Model), e.g. computing magnetic declination.

## C++ Libraries

### magnetic\_model::MagneticModel

A class that can examine the Earth magnetic model and answer questions about it like field strength and components.

### magnetic\_model::MagneticModelManager

A manager class for `MagneticModel`s that can automatically load and return the most suitable model.