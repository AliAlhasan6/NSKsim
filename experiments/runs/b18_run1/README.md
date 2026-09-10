<h1 align="center">Ali Alhasan</h1>

<p align="center">
  <b>PhD Candidate</b> · Neuro-Symbolic Knowledge Systems<br>
  Saint Petersburg Electrotechnical University "LETI" · Specialty 1.2.1
</p>

<p align="center">
  <a href="mailto:aliyossefalhasan@gmail.com">
    <img src="https://img.shields.io/badge/Email-333333?style=flat-square&logo=maildotru&logoColor=white" alt="Email">
  </a>
  <a href="https://orcid.org/0009-0007-1496-2736">
    <img src="https://img.shields.io/badge/ORCID-333333?style=flat-square&logo=orcid&logoColor=white" alt="ORCID">
  </a>
</p>

---

## Research

> Distributed multi-agent knowledge sharing: how autonomous agents exchange structured knowledge under bandwidth constraints when no single agent holds the complete graph.

Supervisor: Dr. Ilya I. Viksnin. Prior degrees: B.Sc. mechanical engineering and metallurgy, M.Sc. automation and mechatronics.

### NSK pipeline

Three-stage architecture for knowledge-graph exchange between agents, evaluated on FB15k-237.
Implementation: [Stage 1](https://github.com/AliAlhasan6/Neural-Knowledge-Compression-NSK-) · [Stage 2](https://github.com/AliAlhasan6/Neural-Knowledge-Compression-Stage-2-NSK_2-)

| Stage | Component | Function |
|:--|:--|:--|
| 1 | Graph compressor | Scores and selects which facts merit transmission |
| 2 | GATv2 embedder | Encodes the selected subgraph |
| 3 | Gated merger | Integrates received knowledge into the local graph |

**Finding.** Two of the compressor's four scoring signals — `score_semantic` and `score_recency` — are inert on FB15k-237. The dataset supplies no textual descriptions and no temporal annotations, so both collapse to constant fallback values, leaving `w_struct` and `w_surp` as the only contributing weights. The sensitivity analysis establishing this is in preparation.

### Manuscripts in preparation

- Neuro-symbolic knowledge graph compression and fusion for distributed multi-agent systems
- Compressor weight-sensitivity analysis (establishing signal inertness on FB15k-237)

---

## Projects

All four run entirely on local hardware — one GTX 1050 Ti, 4 GB VRAM — with no cloud inference at runtime.

<table>
<tr>
<td width="50%" valign="top">

### [NSKsim](https://github.com/AliAlhasan6/NSKsim)

Multi-robot simulation exercising the NSK pipeline under movement and partial observation. 5–8 differential-drive robots exchange compressed knowledge-graph embeddings over ZeroMQ.

`ROS 2 Jazzy` `Gazebo Harmonic` `ZeroMQ` `RViz2`

</td>
<td width="50%" valign="top">

### [MacroVoice](https://github.com/AliAlhasan6/macrovoice)

Local nutrition-tracking assistant on Telegram. Image and voice input, local vision-language model, curated food database, speech output. Deployed as a systemd service. **239 tests passing.**

`LangGraph` `Ollama` `SQLite/FTS5` `Kokoro TTS`

</td>
</tr>
<tr>
<td width="50%" valign="top">

### [CodeForge](https://github.com/AliAlhasan6/codeforge)

Four-agent self-correcting code generation: supervisor, coder, tester, executor, critic, with execution confined to a Docker sandbox. **80% first-iteration pass rate** on a LeetCode-easy benchmark.

`LangGraph` `qwen2.5-coder` `Docker`

</td>
<td width="50%" valign="top">

### [PaperMind](https://github.com/AliAlhasan6/papermind)

Research assistant combining retrieval-augmented generation with an explicit citation knowledge graph, on the reasoning that citation structure carries signal vector similarity discards.

`LangGraph` `ChromaDB` `NetworkX` `Ollama`

</td>
</tr>
</table>

---

## Technical

**Machine learning**
![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=flat-square&logo=pytorch&logoColor=white)
![PyG](https://img.shields.io/badge/PyTorch_Geometric-3C2179?style=flat-square)
![GATv2](https://img.shields.io/badge/GATv2-555555?style=flat-square)
![RAG](https://img.shields.io/badge/RAG-555555?style=flat-square)

**Robotics**
![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat-square&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-1A1A1A?style=flat-square)
![ZeroMQ](https://img.shields.io/badge/ZeroMQ-DF0000?style=flat-square)

**Engineering**
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![SQLite](https://img.shields.io/badge/SQLite-003B57?style=flat-square&logo=sqlite&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-2496ED?style=flat-square&logo=docker&logoColor=white)
![Linux](https://img.shields.io/badge/Linux-FCC624?style=flat-square&logo=linux&logoColor=black)
![Git](https://img.shields.io/badge/Git-F05032?style=flat-square&logo=git&logoColor=white)
