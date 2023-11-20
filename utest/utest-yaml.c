/*******************************************************************************
 * utest-yaml.h
 *
 * IMR unit-test application yaml parser functions
 *
 * Copyright (c) 2018 Cogent Embedded Inc. ALL RIGHTS RESERVED.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#define MODULE_TAG                      YAML_PARSER

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "utest-common.h"
#include "utest-yaml.h"
#include <yaml.h>

/*******************************************************************************
 * Tracing configuration
 ******************************************************************************/

TRACE_TAG(INIT, 1);
TRACE_TAG(INFO, 1);
TRACE_TAG(DEBUG, 0);

/*******************************************************************************
 * Primitives parsing
 ******************************************************************************/

static inline char * parse_token(yaml_document_t *doc, yaml_node_t *v)
{
    CHK_ERR(v && v->type == YAML_SCALAR_NODE, NULL);

    return (char *)v->data.scalar.value;
}

/**
 * Get float type value from yaml field
 * */
static inline int get_float(float *parameter, yaml_document_t *doc, yaml_node_t *value)
{
    int err = 0;
    char *v = NULL;

    v = parse_token(doc, value);

    if (v != NULL)
    {
        char *p = NULL;
        *parameter = strtof(v, &p);

        if (*p != '\0')
        {
            err = -1;
        }
    }
    else
    {
        err = -1;
    }

    return err;
}

/**
 * Get vector of float type values from yaml field
 * */
static inline int get_vector(float *m, int size, yaml_document_t *doc, yaml_node_t *v)
{
    int err = 0;
    yaml_node_item_t   *item;

    if ((v != NULL) && (v->type == YAML_SEQUENCE_NODE))
    {
        int seq_size = v->data.sequence.items.top - v->data.sequence.items.start;

        if (seq_size != size)
        {
            err = -1;
        }
    }

    if (err == 0)
    {
        for (item = v->data.sequence.items.start; size > 0; size--, item++)
        {
            err = get_float(m++, doc, yaml_document_get_node(doc, *item));

            if (err != 0)
            {
                break;
            }
        }
    }

    return err;
}

/* ...matrix parsing */
static yaml_node_t * get_data(yaml_document_t *doc, yaml_node_t *v)
{
    yaml_node_pair_t   *pair;
    yaml_node_t        *data = NULL;


    /* ...retrieve matrix parameters */
    for (pair = v->data.mapping.pairs.start; pair != v->data.mapping.pairs.top; pair++)
    {
        yaml_node_t    *key = yaml_document_get_node(doc, pair->key);
        yaml_node_t    *value = yaml_document_get_node(doc, pair->value);
        char           *t;

        /* ...sanity check */
        BUG(!key || !value, _x("internal error: key=%p, value=%p"), key, value);

        /* ...check key correctness */
        t = parse_token(doc, key);

        /* ...process individual key */
        if (!strcmp(t, "data"))
        {
            data = value;
        }
        else
        {
            TRACE(DEBUG, _b("unrecognized matrix parameter: '%s'"), t);
        }
    }

    return data;
}

/* ...open parser */
static inline int parser_open(yaml_parser_t *p, const char *fname)
{
    FILE   *f;

    TRACE(1, _b("open file: '%s'"), fname);

    /* ...open file descriptor */
    CHK_ERR(f = fopen(fname, "rt"), -errno);

    /* ...open parser */
    if (!yaml_parser_initialize(p))
    {
        TRACE(ERROR, _x("failed to initialize parser"));
        return -(errno = ENOMEM);
    }
    else
    {
        char buf[256];

        /* ...hack to support YAML 1.0 - ugly  */
        if (!fgets(buf, sizeof(buf), f))
        {
            return -1;
        }


        if (strncmp(buf, "%YAML:1.0", 9))
        {
            fseek(f, 0, SEEK_SET);
        }
        else
        {
            /* ...skip second line */
            //fgets(buf, sizeof(buf), f);
        }

        /* ...set input file */
        yaml_parser_set_input_file(p, f);
    }

    TRACE(INIT, _b("use configuration file '%s'"), fname);
    return 0;
}

/*******************************************************************************
 * Tokens parsing
 ******************************************************************************/

/* ...parse camera-intrinsic */
static int get_camera_intrinsic(camera_intrinsic_t *intrinsic, yaml_document_t *doc, yaml_node_t *root)
{
    yaml_node_pair_t   *pair;

    /* ...process camera parameters */
    for (pair = root->data.mapping.pairs.start; pair != root->data.mapping.pairs.top; pair++)
    {
        yaml_node_t    *key = yaml_document_get_node(doc, pair->key);
        yaml_node_t    *value = yaml_document_get_node(doc, pair->value);
        char           *t;

        /* ...sanity check; values must be valid */
        BUG(!key || !value, _x("internal error: key=%p, value=%p"), key, value);

        /* ...process high-level parameters */
        CHK_ERR(t = parse_token(doc, key), -errno);

        /* ...process individual parameters */
        if (!strcmp(t, "CameraMatrix"))
        {
            /* ...parse matrix and convert in to intrinsic structure in opencv format */
            float camera_matrix[9];
            yaml_node_t* data = get_data(doc, value);
            CHK_API(get_vector(camera_matrix, 9, doc, data));
            intrinsic->fx = camera_matrix[0];
            intrinsic->fy = camera_matrix[4];
            intrinsic->cx = camera_matrix[2];
            intrinsic->cy = camera_matrix[5];
        }
        else if (!strcmp(t, "DistortionCoeffs"))
        {
            yaml_node_t* data = get_data(doc, value);
            CHK_API(get_vector(intrinsic->d, 4, doc, data));
        }
        else
        {
            TRACE(DEBUG, _x("unrecognized parameter: '%s'"), t);
        }
    }

    return 0;
}
/* ...parse camera intrinsics parameters */
static inline int parse_camera_intrinsics(camera_intrinsic_t *intrinsic, yaml_document_t *doc, yaml_node_t *root)
{
    /* ...parse parameters */
    CHK_API(get_camera_intrinsic(intrinsic, doc, root));

    return 0;
}

/* ...parsing function for intrinsics */
int config_parse_intrinsics(camera_intrinsic_t *intrinsic, char *fname, int camera_id)
{
    yaml_parser_t   p;
    yaml_node_t    *root;
    int             r;

    /* ...create parser */
    CHK_API(parser_open(&p, fname));

    /* ...go process the file; find all sections */
    do
    {
        yaml_document_t     document;

        /* ...get next document */
        if (!yaml_parser_load(&p, &document))
        {
            goto error;
        }

        /* ...get root document node */
        if ((root = yaml_document_get_root_node(&document)) != NULL)
        {
            if (parse_camera_intrinsics(&intrinsic[camera_id], &document, root) < 0)
            {
                goto error;
            }
        }

        /* ...destroy document */
        yaml_document_delete(&document);
    }
    while (root != NULL);

    TRACE(INIT, _b("parsing successful"));
    r = 0;

out:
    /* ...close parser afterwards */
    yaml_parser_delete(&p);
    return r;

error:
    TRACE(ERROR, _b("configuration parsing failed: %m"));
    r = -errno;
    goto out;
}
