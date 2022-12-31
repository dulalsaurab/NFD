<div align="center">

[<img alt height="70" src="docs/named_data_theme/static/ndn-logo.svg"/>](https://named-data.net/)

# NFD: Named Data Networking Forwarding Daemon

</div>

[![CI](https://github.com/named-data/NFD/actions/workflows/ci.yml/badge.svg)](https://github.com/named-data/NFD/actions/workflows/ci.yml)
[![Docs](https://github.com/named-data/NFD/actions/workflows/docs.yml/badge.svg)](https://github.com/named-data/NFD/actions/workflows/docs.yml)
![Language](https://img.shields.io/badge/C%2B%2B-17-blue)
![Latest version](https://img.shields.io/github/v/tag/named-data/NFD?label=Latest%20version)

## Overview

NFD is a network forwarder that implements and evolves together with the Named Data
Networking (NDN) [protocol](https://named-data.net/doc/NDN-packet-spec/current/).
Since the initial public release in 2014, NFD has been a core component of the
[NDN Platform](https://named-data.net/codebase/platform/).

The main design goal of NFD is to support diverse experimentation of NDN technology.  The
design emphasizes *modularity* and *extensibility* to allow easy experiments with new
protocol features, algorithms, new applications.  We have not fully optimized the code for
performance.  The intention is that performance optimizations are one type of experiments
that developers can conduct by trying out different data structures and different
algorithms; over time, better implementations may emerge within the same design framework.

NFD will keep evolving in three aspects: improvement of the modularity framework, keeping
up with the NDN protocol spec, and addition of other new features. We hope to keep the
modular framework stable and lean, allowing researchers to implement and experiment with
various features, some of which may eventually work into the protocol spec.

## Documentation

See [`docs/INSTALL.rst`](docs/INSTALL.rst) for compilation and installation instructions.

Extensive documentation is available on NFD's [homepage](https://named-data.net/doc/NFD/).

## Reporting bugs

Bug reports and feedback are highly appreciated and can be submitted through the
[NFD issue tracker](https://redmine.named-data.net/projects/nfd/issues) or the
[ndn-interest mailing list](http://www.lists.cs.ucla.edu/mailman/listinfo/ndn-interest).

## Contributing

NFD is developed by a community effort.  Although the first release was mostly done by the
members of [NSF-sponsored NDN project team](https://named-data.net/project/participants/),
it already contains significant contributions from people outside the project team (see
[`AUTHORS.md`](AUTHORS.md)).  We strongly encourage participation from all interested parties,
since broader community support is key for NDN to succeed as a new Internet architecture.

If you are new to the NDN software community, please read [`README-dev.md`](README-dev.md)
and the [Contributor's Guide](https://github.com/named-data/.github/blob/master/CONTRIBUTING.md)
to get started.

## License

NFD is a free and open-source software package licensed under the GPL version 3 and
is the centerpiece of our committement to making NDN's core technology free and open
to all Internet users and developers. For more information about licensing, refer to
[`COPYING.md`](COPYING.md).
