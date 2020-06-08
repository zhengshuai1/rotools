from __future__ import print_function


class WebotsConverter(object):
    """This tool converts solid mesh in Webots proto file to stl format in
    a semi-automatic way. First, you need copy the vertexes and coordinate index
    of a mesh to x.vertex and x.index manually, and then using this tool to
    generate stl in ASCII format. You may need to further convert the stl file
    to binary using Blender to save space.
    """

    def __init__(self):
        super(WebotsConverter, self).__init__()
        self.vertexes_list = None
        self.face_list = []

    def read_proto(self, vertex_file, index_file):
        """Given a webots proto, you need manually copy the coord Coordinate ->
        point and coordIndex values into file vertex_file and index_file, respectively.
        Remove the last ', -1' in index_file.

        :param vertex_file: string file contains all vertexes coordinates
        :param index_file: string file records vertex index of each face
        :return:
        """
        self.face_list = []

        with open(vertex_file, 'r') as v_file:
            vertexes = v_file.readline()
            self.vertexes_list = vertexes.split(', ')
            print('Total vertexes #', len(self.vertexes_list))

        with open(index_file, 'r') as i_file:
            indexes = i_file.readline()
            index_list = indexes.split(', -1, ')

            for i in index_list:
                # index of the vertexes belonging to the same face
                i1, i2, i3 = i.split(', ')
                v1 = self.vertexes_list[int(i1)]
                v2 = self.vertexes_list[int(i2)]
                v3 = self.vertexes_list[int(i3)]
                self.face_list.append([v1, v2, v3])

    def write_to_stl(self, stl_file):
        """

        facet normal -1.000000 0.000000 0.000000
            outer loop
            vertex -1.000000 -1.000000 -1.000000
            vertex -1.000000 -1.000000 1.000000
            vertex -1.000000 1.000000 1.000000
            endloop
        endfacet

        :param stl_file:
        :return:
        """
        with open(stl_file, 'w') as s_file:
            s_file.write('solid Exported from RoTools\n')
            for face in self.face_list:
                # s_file.write('face normal {} {} {}'.format(nx, ny, nz))
                s_file.write('outer loop\n')
                s_file.write('vertex {}\n'.format(face[0]))
                s_file.write('vertex {}\n'.format(face[1]))
                s_file.write('vertex {}\n'.format(face[2]))
                s_file.write('endloop\n')
                s_file.write('endfacet\n')
            s_file.write('endsolid Exported from RoTools')

