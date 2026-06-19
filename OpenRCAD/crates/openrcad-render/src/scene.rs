//! Uploading an [`openrcad_mesh::GpuMesh`] into wgpu buffers.
//!
//! The flat-shaded buffers are interleaved into a single
//! `position + normal + face_id` vertex stream and uploaded once. Keeping the
//! face id in the vertex stream lets the shader highlight selected topology
//! without a second draw pass.

use openrcad_mesh::GpuMesh;
use wgpu::util::DeviceExt;

/// One interleaved vertex: position, normal, then source face id.
pub const VERTEX_STRIDE: wgpu::BufferAddress =
    (7 * std::mem::size_of::<f32>()) as wgpu::BufferAddress;

/// The wgpu vertex layout matching [`VERTEX_STRIDE`] and `shader.wgsl`.
pub fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![
        0 => Float32x3, // position
        1 => Float32x3, // normal
        2 => Float32,   // source face id
    ];
    wgpu::VertexBufferLayout {
        array_stride: VERTEX_STRIDE,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

/// The wgpu vertex layout for the wireframe overlay: a single `vec3` position.
pub fn edge_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![0 => Float32x3];
    wgpu::VertexBufferLayout {
        array_stride: (3 * std::mem::size_of::<f32>()) as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

/// GPU-resident geometry for a single mesh.
pub struct SceneMesh {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    /// Line-list vertex buffer of the model's topological edges.
    pub edge_buffer: wgpu::Buffer,
    /// Number of edge-overlay vertices (2 per segment).
    pub edge_vertex_count: u32,
    /// Per-triangle source face index (CPU side, for picking).
    pub face_ids: Vec<u32>,
}

impl SceneMesh {
    /// Interleave and upload a [`GpuMesh`].
    pub fn upload(device: &wgpu::Device, mesh: &GpuMesh) -> Self {
        let vertex_count = mesh.positions.len() / 3;
        let mut interleaved = Vec::with_capacity(vertex_count * 7);
        for i in 0..vertex_count {
            interleaved.extend_from_slice(&mesh.positions[i * 3..i * 3 + 3]);
            interleaved.extend_from_slice(&mesh.normals[i * 3..i * 3 + 3]);
            let tri = i / 3;
            let face_id = mesh.face_ids.get(tri).copied().unwrap_or(0) as f32;
            interleaved.push(face_id);
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("openrcad-render vertices"),
            contents: bytemuck::cast_slice(&interleaved),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("openrcad-render indices"),
            contents: bytemuck::cast_slice(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        let edge_lines = crate::edges::feature_edge_lines(mesh);
        let edge_vertex_count = (edge_lines.len() / 3) as u32;
        // wgpu rejects zero-sized buffers, so a model with no edges gets a tiny
        // placeholder that is simply never drawn (edge_vertex_count == 0).
        let edge_contents: &[f32] = if edge_lines.is_empty() {
            &[0.0, 0.0, 0.0]
        } else {
            &edge_lines
        };
        let edge_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("openrcad-render edges"),
            contents: bytemuck::cast_slice(edge_contents),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            vertex_buffer,
            index_buffer,
            index_count: mesh.indices.len() as u32,
            edge_buffer,
            edge_vertex_count,
            face_ids: mesh.face_ids.clone(),
        }
    }
}
